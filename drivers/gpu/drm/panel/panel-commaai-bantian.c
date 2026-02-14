// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Robin Reckmann <robin.reckmann@gmail.com>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct commaai_bantian {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator *vddio;
	struct regulator *vdd;
	enum drm_panel_orientation orientation;
};

static inline struct commaai_bantian *
to_commaai_bantian(struct drm_panel *panel)
{
	return container_of(panel, struct commaai_bantian, panel);
}

static void commaai_bantian_reset(struct commaai_bantian *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int commaai_bantian_power_on(struct commaai_bantian *ctx)
{
	int ret;

	ret = regulator_enable(ctx->vddio);
	if (ret)
		return ret;
	usleep_range(3000, 5000);

	ret = regulator_enable(ctx->vdd);
	if (ret) {
		regulator_disable(ctx->vddio);
		return ret;
	}
	usleep_range(3000, 5000);
	return 0;
}

static void commaai_bantian_power_off(struct commaai_bantian *ctx)
{
	regulator_disable(ctx->vdd);
	regulator_disable(ctx->vddio);
}

static int commaai_bantian_init_sequence(struct commaai_bantian *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x99);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x00);

	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0x77); /* 24-bit */
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0000);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x00);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx,
				       MIPI_DSI_DCS_TEAR_MODE_VHBLANK);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x03);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	return dsi_ctx.accum_err;
}

static int commaai_bantian_prepare(struct drm_panel *panel)
{
	struct commaai_bantian *ctx = to_commaai_bantian(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = commaai_bantian_power_on(ctx);
	if (ret) {
		dev_err(dev, "Failed to power on: %d\n", ret);
		return ret;
	}

	commaai_bantian_reset(ctx);

	ret = commaai_bantian_init_sequence(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		commaai_bantian_power_off(ctx);
		return ret;
	}

	return 0;
}

static int commaai_bantian_unprepare(struct drm_panel *panel)
{
	struct commaai_bantian *ctx = to_commaai_bantian(panel);
	struct mipi_dsi_multi_context c = { .dsi = ctx->dsi };

	mipi_dsi_dcs_enter_sleep_mode_multi(&c);
	mipi_dsi_msleep(&c, 120);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return c.accum_err;
}

static int commaai_bantian_enable(struct drm_panel *panel)
{
	struct commaai_bantian *ctx = to_commaai_bantian(panel);
	struct mipi_dsi_multi_context c = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_on_multi(&c);
	mipi_dsi_msleep(&c, 20);

	return c.accum_err;
}

static int commaai_bantian_disable(struct drm_panel *panel)
{
	struct commaai_bantian *ctx = to_commaai_bantian(panel);
	struct mipi_dsi_multi_context c = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&c);
	mipi_dsi_usleep_range(&c, 5000, 10000);

	return c.accum_err;
}

static const struct drm_display_mode commaai_bantian_mode = {
	.clock = (240 + 20 + 20 + 40) * (536 + 20 + 4 + 12) * 60 / 1000,
	.hdisplay = 240,
	.hsync_start = 240 + 20,
	.hsync_end = 240 + 20 + 20,
	.htotal = 240 + 20 + 20 + 40,
	.vdisplay = 536,
	.vsync_start = 536 + 20,
	.vsync_end = 536 + 20 + 4,
	.vtotal = 536 + 20 + 4 + 12,
	.width_mm = 0,
	.height_mm = 0,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int commaai_bantian_get_modes(struct drm_panel *panel,
				     struct drm_connector *connector)
{
	connector->display_info.bpc = 8;
	return drm_connector_helper_get_modes_fixed(connector,
						    &commaai_bantian_mode);
}

static enum drm_panel_orientation
commaai_bantian_get_orientation(struct drm_panel *panel)
{
	struct commaai_bantian *ctx = to_commaai_bantian(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs commaai_bantian_panel_funcs = {
	.prepare = commaai_bantian_prepare,
	.unprepare = commaai_bantian_unprepare,
	.enable = commaai_bantian_enable,
	.disable = commaai_bantian_disable,
	.get_modes = commaai_bantian_get_modes,
	.get_orientation = commaai_bantian_get_orientation,
};

static int commaai_bantian_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static const struct backlight_ops commaai_bantian_bl_ops = {
	.update_status = commaai_bantian_bl_update_status,
};

static struct backlight_device *
commaai_bantian_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &commaai_bantian_bl_ops, &props);
}

static int commaai_bantian_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct commaai_bantian *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct commaai_bantian, panel,
				   &commaai_bantian_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio),
				     "Failed to get vddio-supply\n");

	ctx->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctx->vdd))
		return dev_err_probe(dev, PTR_ERR(ctx->vdd),
				     "Failed to get vdd-supply\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 1;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = commaai_bantian_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret) {
		return dev_err_probe(dev, ret, "Failed to get orientation\n");
	}

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret,
				     "Failed to attach to DSI host\n");
	}

	return 0;
}

static void commaai_bantian_remove(struct mipi_dsi_device *dsi)
{
	struct commaai_bantian *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id commaai_bantian_of_match[] = {
	{ .compatible = "commaai,bantian" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, commaai_bantian_of_match);

static struct mipi_dsi_driver commaai_bantian_driver = {
	.probe = commaai_bantian_probe,
	.remove = commaai_bantian_remove,
	.driver = {
		.name = "panel-commaai-bantian",
		.of_match_table = commaai_bantian_of_match,
	},
};
module_mipi_dsi_driver(commaai_bantian_driver);

MODULE_AUTHOR("Robin Reckmann <robin.reckmann@gmail.com>");
MODULE_DESCRIPTION("DRM driver for comma.ai bantian panel");
MODULE_LICENSE("GPL");