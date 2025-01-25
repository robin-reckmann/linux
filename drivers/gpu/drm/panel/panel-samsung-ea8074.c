// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved.

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct samsung_ea8074 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline struct samsung_ea8074 *to_samsung_ea8074(struct drm_panel *panel)
{
	return container_of(panel, struct samsung_ea8074, panel);
}

static void samsung_ea8074_reset(struct samsung_ea8074 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int samsung_ea8074_on(struct samsung_ea8074 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_set_column_address(dsi, 0x0000, 0x0437);
	if (ret < 0) {
		dev_err(dev, "Failed to set column address: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);


	ret = mipi_dsi_dcs_set_page_address(dsi, 0x0000, 0x086f);
	if (ret < 0) {
		dev_err(dev, "Failed to set page address: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x0000);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x06);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xef, 0x35);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xcc, 0x55, 0x12);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xfc, 0x5a, 0x5a);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x01);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xd2, 0x20);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x05);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xd2, 0x40);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xfc, 0xa5, 0xa5);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	msleep(110);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_MEMORY_START);
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x05);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xb1, 0x40);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x03);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xb6, 0xa2);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	msleep(120);

	return 0;
}

static int samsung_ea8074_off(struct samsung_ea8074 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	return 0;
}

static int samsung_ea8074_prepare(struct drm_panel *panel)
{
	struct samsung_ea8074 *ctx = to_samsung_ea8074(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	samsung_ea8074_reset(ctx);

	ret = samsung_ea8074_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int samsung_ea8074_unprepare(struct drm_panel *panel)
{
	struct samsung_ea8074 *ctx = to_samsung_ea8074(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = samsung_ea8074_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode samsung_ea8074_mode = {
	.clock = (1080 + 48 + 16 + 48) * (2160 + 20 + 12 + 28) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 48,
	.hsync_end = 1080 + 48 + 16,
	.htotal = 1080 + 48 + 16 + 48,
	.vdisplay = 2160,
	.vsync_start = 2160 + 20,
	.vsync_end = 2160 + 20 + 12,
	.vtotal = 2160 + 20 + 12 + 28,
	.width_mm = 68,
	.height_mm = 137,
};

static int samsung_ea8074_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &samsung_ea8074_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs samsung_ea8074_panel_funcs = {
	.prepare = samsung_ea8074_prepare,
	.unprepare = samsung_ea8074_unprepare,
	.get_modes = samsung_ea8074_get_modes,
};

static int samsung_ea8074_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int samsung_ea8074_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops samsung_ea8074_bl_ops = {
	.update_status = samsung_ea8074_bl_update_status,
	.get_brightness = samsung_ea8074_bl_get_brightness,
};

static struct backlight_device *
samsung_ea8074_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1023,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &samsung_ea8074_bl_ops, &props);
}

static int samsung_ea8074_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct samsung_ea8074 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &samsung_ea8074_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = samsung_ea8074_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void samsung_ea8074_remove(struct mipi_dsi_device *dsi)
{
	struct samsung_ea8074 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id samsung_ea8074_of_match[] = {
	{ .compatible = "samsung,ea8074" },
	{},
};
MODULE_DEVICE_TABLE(of, samsung_ea8074_of_match);

static struct mipi_dsi_driver samsung_ea8074_driver = {
	.probe = samsung_ea8074_probe,
	.remove = samsung_ea8074_remove,
	.driver = {
		.name = "panel-samsung-ea8074",
		.of_match_table = samsung_ea8074_of_match,
	},
};
module_mipi_dsi_driver(samsung_ea8074_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator");
MODULE_DESCRIPTION("DRM driver for samsung fhd cmd dsi panel");
MODULE_LICENSE("GPL");
