#include <linux/clk.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define PIXEL_RATE 88000000ULL

/* Chip ID */
#define AR0231_REG_CHIP_ID CCI_REG16(0x3000)
#define AR0231_CHIP_ID 0x0354

#define AR0231_COARSE_INTEGRATION_TIME CCI_REG16(0x3012)

#define AR0231_ANALOG_GAIN CCI_REG16(0x3366)

#define AR0231_REG_MODE_SELECT CCI_REG16(0x301a)
#define AR0231_MODE_RESET 0x0018
#define AR0231_MODE_STANDBY 0x0918
#define AR0231_MODE_STREAMING 0x091C

#define AR0231_REG_VTS CCI_REG16(0x300a)
#define AR0231_VTS_MAX 0xffff

#define AR0231_NATIVE_WIDTH 1928
#define AR0231_NATIVE_HEIGHT 1208

#define to_ar0231(_sd) container_of(_sd, struct ar0231, sd)

struct ar0231_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct ar0231_mode {
	u32 width;
	u32 height;

	u32 link_freq_index;
	u32 code;

	u32 hblank;
	u32 vblank;

	/* Sensor register settings for this mode */
	const struct ar0231_reg_list reg_list;
};

static const struct cci_reg_sequence mode_default[] = {

	// CLOCK Settings
	{ CCI_REG16(0x302a), 0x0006 }, // VT_PIX_CLK_DIV
	{ CCI_REG16(0x302c), 0x0001 }, // VT_SYS_CLK_DIV
	{ CCI_REG16(0x302e), 0x0002 }, // PRE_PLL_CLK_DIV
	{ CCI_REG16(0x3030), 0x0037 }, // PLL_MULTIPLIER
	{ CCI_REG16(0x3036), 0x000C }, // OP_PIX_CLK_DIV
	{ CCI_REG16(0x3038), 0x0001 }, // OP_SYS_CLK_DIV

	// FORMAT
	{ CCI_REG16(0x3040), 0xC000 }, // READ_MODE
	{ CCI_REG16(0x3004), 0x0000 }, // X_ADDR_START_
	{ CCI_REG16(0x3008), 0x0787 }, // X_ADDR_END_
	{ CCI_REG16(0x3002), 0x0000 }, // Y_ADDR_START_
	{ CCI_REG16(0x3006), 0x04B7 }, // Y_ADDR_END_
	{ CCI_REG16(0x3032), 0x0000 }, // SCALING_MODE
	{ CCI_REG16(0x30A2), 0x0001 }, // X_ODD_INC_
	{ CCI_REG16(0x30A6), 0x0001 }, // Y_ODD_INC_
	{ CCI_REG16(0x3402), 0x0788 }, // X_OUTPUT_CONTROL
	{ CCI_REG16(0x3404), 0x04B8 }, // Y_OUTPUT_CONTROL
	{ CCI_REG16(0x3064), 0x1982 }, // SMIA_TEST
	{ CCI_REG16(0x30BA), 0x11F2 }, // DIGITAL_CTRL

	// Enable external trigger and disable GPIO outputs
	// { CCI_REG16(0x30CE), 0x0120}, // SLAVE_SH_SYNC_MODE | FRAME_START_MODE
	{ CCI_REG16(0x340A),
	  0xE0 }, // GPIO3_INPUT_DISABLE | GPIO2_INPUT_DISABLE | GPIO1_INPUT_DISABLE
	{ CCI_REG16(0x340C), 0x802 }, // GPIO_HIDRV_EN | GPIO0_ISEL=2

	// Readout timing
	{ CCI_REG16(0x300C),
	  0x0672 }, // LINE_LENGTH_PCK (valid for 3-exposure HDR)
	{ CCI_REG16(0x300A), 0x0855 }, // FRAME_LENGTH_LINES
	{ CCI_REG16(0x3042), 0x0000 }, // EXTRA_DELAY

	// Readout Settings
	{ CCI_REG16(0x31AE), 0x0204 }, // SERIAL_FORMAT, 4-lane MIPI
	{ CCI_REG16(0x31AC), 0x0C0C }, // DATA_FORMAT_BITS, 12 -> 12
	{ CCI_REG16(0x3342), 0x122C }, // MIPI_F1_PDT_EDT
	{ CCI_REG16(0x3346), 0x122C }, // MIPI_F2_PDT_EDT
	{ CCI_REG16(0x334A), 0x122C }, // MIPI_F3_PDT_EDT
	{ CCI_REG16(0x334E), 0x122C }, // MIPI_F4_PDT_EDT
	{ CCI_REG16(0x3344), 0x0011 }, // MIPI_F1_VDT_VC
	{ CCI_REG16(0x3348), 0x0011 }, // MIPI_F2_VDT_VC
	{ CCI_REG16(0x334C), 0x0011 }, // MIPI_F3_VDT_VC
	{ CCI_REG16(0x3350), 0x0011 }, // MIPI_F4_VDT_VC
	{ CCI_REG16(0x31B0), 0x0053 }, // FRAME_PREAMBLE
	{ CCI_REG16(0x31B2), 0x003B }, // LINE_PREAMBLE

	// Noise Corrections
	{ CCI_REG16(0x3092), 0x0C24 }, // ROW_NOISE_CONTROL
	{ CCI_REG16(0x337A), 0x0C80 }, // DBLC_SCALE0
	{ CCI_REG16(0x3370), 0x03B1 }, // DBLC
	{ CCI_REG16(0x3044), 0x0400 }, // DARK_CONTROL

	// Enable temperature sensor
	{ CCI_REG16(0x30B4), 0x0007 }, // TEMPSENS0_CTRL_REG
	{ CCI_REG16(0x30B8), 0x0007 }, // TEMPSENS1_CTRL_REG

	// Enable dead pixel correction using
	// the 1D line correction scheme
	{ CCI_REG16(0x31E0), 0x0003 },

	// HDR Settings
	{ CCI_REG16(0x3082), 0x0004 }, // OPERATION_MODE_CTRL
	{ CCI_REG16(0x3238), 0x0444 }, // EXPOSURE_RATIO

	{ CCI_REG16(0x1008), 0x0361 }, // FINE_INTEGRATION_TIME_MIN
	{ CCI_REG16(0x100C), 0x0589 }, // FINE_INTEGRATION_TIME2_MIN
	{ CCI_REG16(0x100E), 0x07B1 }, // FINE_INTEGRATION_TIME3_MIN
	{ CCI_REG16(0x1010), 0x0139 }, // FINE_INTEGRATION_TIME4_MIN

	// TODO: do these have to be lower than LINE_LENGTH_PCK?
	{ CCI_REG16(0x3014), 0x08CB }, // FINE_INTEGRATION_TIME_
	{ CCI_REG16(0x321E), 0x0894 }, // FINE_INTEGRATION_TIME2

	{ CCI_REG16(0x31D0), 0x0000 }, // COMPANDING, no good in 10 bit?
	{ CCI_REG16(0x33DA), 0x0000 }, // COMPANDING
	{ CCI_REG16(0x318E), 0x0200 }, // PRE_HDR_GAIN_EN

	// DLO Settings
	{ CCI_REG16(0x3100), 0x4000 }, // DLO_CONTROL0
	{ CCI_REG16(0x3280), 0x0CCC }, // T1 G1
	{ CCI_REG16(0x3282), 0x0CCC }, // T1 R
	{ CCI_REG16(0x3284), 0x0CCC }, // T1 B
	{ CCI_REG16(0x3286), 0x0CCC }, // T1 G2
	{ CCI_REG16(0x3288), 0x0FA0 }, // T2 G1
	{ CCI_REG16(0x328A), 0x0FA0 }, // T2 R
	{ CCI_REG16(0x328C), 0x0FA0 }, // T2 B
	{ CCI_REG16(0x328E), 0x0FA0 }, // T2 G2

	// Initial Gains
	{ CCI_REG16(0x3022), 0x0001 }, // GROUPED_PARAMETER_HOLD_
	{ CCI_REG16(0x3366), 0xFF77 }, // ANALOG_GAIN (1x)

	{ CCI_REG16(0x3060), 0x3333 }, // ANALOG_COLOR_GAIN

	{ CCI_REG16(0x3362), 0x0001 }, // DC GAIN

	{ CCI_REG16(0x305A), 0x00F8 }, // red gain
	{ CCI_REG16(0x3058), 0x0122 }, // blue gain
	{ CCI_REG16(0x3056), 0x009A }, // g1 gain
	{ CCI_REG16(0x305C), 0x009A }, // g2 gain

	{ CCI_REG16(0x3022), 0x0000 }, // GROUPED_PARAMETER_HOLD_

	// Initial Integration Time
	{ CCI_REG16(0x3012), 0x0005 },
};

static const s64 link_freq_menu_items[] = {
	528000000ULL,
};

static const struct ar0231_mode ar0231_modes[] = {
	{
		.width = AR0231_NATIVE_WIDTH,
		.height = AR0231_NATIVE_HEIGHT,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_default),
			.regs = mode_default,
		},
		.link_freq_index = 0,
		.hblank = 0,
		.vblank = 0,
	},
};

static const char *const ar0231_supplies[] = {
	"vaa", /* Analog supply */
	"vdd_io", /* I/O Digital supply */
	"vdd", /* Core Digital supply */
};

struct ar0231 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;

	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;

	struct device *dev;
	struct regmap *cci;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ar0231_supplies)];
	unsigned long link_freq_bitmap;
	const struct ar0231_mode *cur_mode;
};

static int ar0231_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0231 *ar0231 =
		container_of(ctrl->handler, struct ar0231, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ar0231->sd);
	int ret;

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(ar0231->cci, AR0231_ANALOG_GAIN,
				0xFF00 | ctrl->val << 4 | ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(ar0231->cci, AR0231_REG_VTS,
				ar0231->cur_mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(ar0231->cci, AR0231_COARSE_INTEGRATION_TIME,
				ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0231_ctrl_ops = {
	.s_ctrl = ar0231_set_ctrl,
};

static int ar0231_init_controls(struct ar0231 *ar0231)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0231->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 vblank, hblank;
	u32 link_freq_size;
	int ret;

	ctrl_hdlr = &ar0231->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	link_freq_size = ARRAY_SIZE(link_freq_menu_items) - 1;
	ar0231->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &ar0231_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   link_freq_size, 0,
						   link_freq_menu_items);
	if (ar0231->link_freq)
		ar0231->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ar0231->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ar0231_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, PIXEL_RATE,
					       PIXEL_RATE, 1, PIXEL_RATE);
	if (ar0231->pixel_rate)
		ar0231->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = AR0231_VTS_MAX - ar0231->cur_mode->height;
	ar0231->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0231_ctrl_ops,
					   V4L2_CID_VBLANK, 0, vblank, 1,
					   ar0231->cur_mode->vblank);
	hblank = ar0231->cur_mode->hblank;
	ar0231->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0231_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (ar0231->hblank)
		ar0231->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0231_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  0, 15, 1, 7);

	v4l2_ctrl_new_std(ctrl_hdlr, &ar0231_ctrl_ops, V4L2_CID_EXPOSURE, 2,
			  0x0855, 1, 5);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		return ret;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ar0231_ctrl_ops,
					      &props);
	if (ret)
		return ret;

	ar0231->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static void ar0231_update_pad_format(const struct ar0231_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int ar0231_start_streaming(struct ar0231 *ar0231)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0231->sd);
	const struct ar0231_reg_list *reg_list;
	int link_freq_index, ret;
	u64 val;

	ret = cci_write(ar0231->cci, AR0231_REG_MODE_SELECT, AR0231_MODE_RESET,
			NULL);
	if (ret) {
		dev_err(&client->dev, "failed to reset");
		return ret;
	}

	reg_list = &ar0231->cur_mode->reg_list;
	ret = cci_multi_reg_write(ar0231->cci, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to set mode");
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(ar0231->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = cci_write(ar0231->cci, AR0231_REG_MODE_SELECT,
			AR0231_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to start stream");
		return ret;
	}

	return 0;
}

static int ar0231_stop_streaming(struct ar0231 *ar0231)
{
	return cci_write(ar0231->cci, AR0231_REG_MODE_SELECT,
			 AR0231_MODE_STANDBY, NULL);
}

static int ar0231_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0231 *ar0231 = to_ar0231(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			return ret;

		ret = ar0231_start_streaming(ar0231);
		if (ret)
			goto error_rpm_put;
	} else {
		ar0231_stop_streaming(ar0231);
		pm_runtime_put(&client->dev);
	}

	return ret;

error_rpm_put:
	pm_runtime_put(&client->dev);

	return ret;
}

static int ar0231_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0231 *ar0231 = to_ar0231(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		ar0231_update_pad_format(ar0231->cur_mode, &fmt->format);
	}

	return 0;
}

static int ar0231_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0231 *ar0231 = to_ar0231(sd);
	struct v4l2_rect *crop;
	const struct ar0231_mode *mode;
	s64 hblank;
	int ret;

	mode = v4l2_find_nearest_size(ar0231_modes, ARRAY_SIZE(ar0231_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	ar0231_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
	} else {
		ar0231->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(ar0231->link_freq, mode->link_freq_index);

		hblank = ar0231->cur_mode->hblank;
		__v4l2_ctrl_modify_range(ar0231->hblank, hblank, hblank, 1,
					 hblank);

		__v4l2_ctrl_modify_range(ar0231->vblank, 0,
					 AR0231_VTS_MAX - mode->height, 1,
					 ar0231->cur_mode->vblank);
		__v4l2_ctrl_s_ctrl(ar0231->vblank, ar0231->cur_mode->vblank);
	}

	return 0;
}

static int ar0231_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG12_1X12;

	return 0;
}

static int ar0231_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(ar0231_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG12_1X12)
		return -EINVAL;

	fse->min_width = ar0231_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ar0231_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ar0231_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = AR0231_NATIVE_WIDTH;
		sel->r.height = AR0231_NATIVE_HEIGHT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ar0231_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	const struct ar0231_mode *def_mode = &ar0231_modes[0];
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = {
			.code = MEDIA_BUS_FMT_SGRBG12_1X12,
			.width = def_mode->width,
			.height = def_mode->height,
		},
	};

	ar0231_set_fmt(sd, sd_state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops ar0231_video_ops = {
	.s_stream = ar0231_set_stream,
};

static const struct v4l2_subdev_pad_ops ar0231_pad_ops = {
	.enum_mbus_code = ar0231_enum_mbus_code,
	.enum_frame_size = ar0231_enum_frame_size,
	.get_fmt = ar0231_get_fmt,
	.set_fmt = ar0231_set_fmt,
	.get_selection = ar0231_get_selection,
};

static const struct v4l2_subdev_core_ops ar0231_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops ar0231_subdev_ops = {
	.core = &ar0231_core_ops,
	.video = &ar0231_video_ops,
	.pad = &ar0231_pad_ops,
};

static const struct media_entity_operations ar0231_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops ar0231_internal_ops = {
	.init_state = ar0231_init_state,
};

static int ar0231_parse_hw_config(struct ar0231 *ar0231)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int i;
	int ret;

	ar0231->clk = devm_clk_get(ar0231->dev, NULL);
	if (IS_ERR(ar0231->clk))
		return dev_err_probe(ar0231->dev, PTR_ERR(ar0231->clk),
				     "Failed to get clock\n");

	ar0231->reset_gpio =
		devm_gpiod_get_optional(ar0231->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ar0231->reset_gpio))
		return dev_err_probe(ar0231->dev, PTR_ERR(ar0231->reset_gpio),
				     "Failed to get reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(ar0231_supplies); i++)
		ar0231->supplies[i].supply = ar0231_supplies[i];

	ret = devm_regulator_bulk_get(ar0231->dev, ARRAY_SIZE(ar0231_supplies),
				      ar0231->supplies);
	if (ret)
		return dev_err_probe(ar0231->dev, ret, "Cannot get supplies\n");

	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(ar0231->dev), NULL);
	if (!endpoint) {
		dev_err(ar0231->dev, "endpoint node not found\n");
		return -EPROBE_DEFER;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg);
	if (ret) {
		dev_err(ar0231->dev, "parsing endpoint node failed\n");
		goto error_out;
	}

	ret = v4l2_link_freq_to_bitmap(ar0231->dev, ep_cfg.link_frequencies,
				       ep_cfg.nr_of_link_frequencies,
				       link_freq_menu_items,
				       ARRAY_SIZE(link_freq_menu_items),
				       &ar0231->link_freq_bitmap);
	if (ret)
		goto error_out;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);
	return ret;
}

static int ar0231_identify_module(struct ar0231 *ar0231)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0231->sd);
	int ret;
	u64 val;

	ret = cci_read(ar0231->cci, AR0231_REG_CHIP_ID, &val, NULL);
	if (ret)
		return ret;

	if (val != AR0231_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%llx",
			AR0231_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

static int ar0231_power_on(struct device *dev)
{
	unsigned long clk_rate;
	int ret;

	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0231 *sensor = to_ar0231(sd);

	if (sensor->reset_gpio)
		gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(ar0231_supplies),
				    sensor->supplies);
	if (ret) {
		dev_err(sensor->dev, "Failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(sensor->clk);
	if (ret) {
		dev_err(sensor->dev, "Failed to enable clock\n");
		goto error_reset;
	}

	if (sensor->reset_gpio)
		gpiod_set_value_cansleep(sensor->reset_gpio, 0);

  	// wait 650000 cycles @ 19.2 mhz = 33.8 ms
	fsleep(33800);

	return ret;

error_reset:
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ar0231_supplies), sensor->supplies);

	return ret;
}

static int ar0231_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0231 *sensor = to_ar0231(sd);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	clk_disable_unprepare(sensor->clk);
	regulator_bulk_disable(ARRAY_SIZE(ar0231_supplies), sensor->supplies);

	return 0;
}

static int ar0231_probe(struct i2c_client *client)
{
	struct ar0231 *ar0231;
	unsigned int i;
	int ret;

	ar0231 = devm_kzalloc(&client->dev, sizeof(*ar0231), GFP_KERNEL);
	if (!ar0231)
		return -ENOMEM;

	ar0231->dev = &client->dev;

	ar0231->cci = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ar0231->cci))
		return dev_err_probe(ar0231->dev, PTR_ERR(ar0231->cci),
				     "Failed to initialize CCI\n");

	v4l2_i2c_subdev_init(&ar0231->sd, client, &ar0231_subdev_ops);

	ret = ar0231_parse_hw_config(ar0231);
	if (ret)
		return ret;

	ret = ar0231_power_on(ar0231->dev);
	if (ret) {
		dev_err_probe(ar0231->dev, ret,
			      "Could not power on the device\n");
		return ret;
	}

	ret = ar0231_identify_module(ar0231);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		return ret;
	}

	ar0231->cur_mode = &ar0231_modes[0];
	ret = ar0231_init_controls(ar0231);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	/* Initialize subdev */
	ar0231->sd.internal_ops = &ar0231_internal_ops;
	ar0231->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	ar0231->sd.entity.ops = &ar0231_subdev_entity_ops;
	ar0231->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	ar0231->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ar0231->sd.entity, 1, &ar0231->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ar0231->sd.state_lock = ar0231->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&ar0231->sd);
	if (ret < 0) {
		dev_err(ar0231->dev, "v4l2 subdev init error: %d\n", ret);
		goto probe_error_media_entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&ar0231->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto probe_error_rpm;
	}

	pm_runtime_set_active(ar0231->dev);
	pm_runtime_enable(ar0231->dev);
	pm_runtime_idle(ar0231->dev);
	
	return 0;

probe_error_rpm:
	pm_runtime_disable(ar0231->dev);
	v4l2_subdev_cleanup(&ar0231->sd);

probe_error_media_entity_cleanup:
	media_entity_cleanup(&ar0231->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(ar0231->sd.ctrl_handler);

	return ret;
}

static void ar0231_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0231 *ar0231 = to_ar0231(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ar0231_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id ar0231_of_match[] = {
	{ .compatible = "onnn,ar0231" },
	{}
};
MODULE_DEVICE_TABLE(of, ar0231_of_match);

static const struct dev_pm_ops ar0231_pm_ops = { SET_RUNTIME_PM_OPS(
	ar0231_power_off, ar0231_power_on, NULL) };

static struct i2c_driver ar0231_i2c_driver = {
	.driver = {
		.name = "ar0231",
		.of_match_table = of_match_ptr(ar0231_of_match),
		.pm = &ar0231_pm_ops,
	},
	.probe = ar0231_probe,
	.remove = ar0231_remove,
};

module_i2c_driver(ar0231_i2c_driver);

MODULE_DESCRIPTION("ON Semiconductor ar0231 sensor driver");
MODULE_AUTHOR("Robin Reckmann <robin.reckmann@gmail.com");
MODULE_LICENSE("GPL v2");
