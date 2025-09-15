// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2025 Robin Reckmann <robin.reckmann@gmail.com>

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

struct samsung_ea8074 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator *vddio;
	struct regulator *vdd;
	enum drm_panel_orientation orientation;
};

static inline struct samsung_ea8074 *to_samsung_ea8074(struct drm_panel *panel)
{
	return container_of(panel, struct samsung_ea8074, panel);
}

static void samsung_ea8074_reset(struct samsung_ea8074 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int samsung_ea8074_power_on(struct samsung_ea8074 *ctx)
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

static void samsung_ea8074_power_off(struct samsung_ea8074 *ctx)
{
	regulator_disable(ctx->vdd);
	regulator_disable(ctx->vddio);
}

static int samsung_ea8074_init_sequence(struct samsung_ea8074 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, MIPI_DCS_PIXEL_FMT_24BIT);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0x0000, 0x0437);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0x0000, 0x086f);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xef, 0x35);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x55, 0x12);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x40);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	return dsi_ctx.accum_err;
}

static int samsung_ea8074_prepare(struct drm_panel *panel)
{
	struct samsung_ea8074 *ctx = to_samsung_ea8074(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = samsung_ea8074_power_on(ctx);
	if (ret) {
		dev_err(dev, "Failed to power on: %d\n", ret);
		return ret;
	}

	samsung_ea8074_reset(ctx);

	ret = samsung_ea8074_init_sequence(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		samsung_ea8074_power_off(ctx);
		return ret;
	}

	return 0;
}

static int samsung_ea8074_unprepare(struct drm_panel *panel)
{
	struct samsung_ea8074 *ctx = to_samsung_ea8074(panel);
	struct mipi_dsi_multi_context c = { .dsi = ctx->dsi };

	mipi_dsi_dcs_enter_sleep_mode_multi(&c);
	mipi_dsi_msleep(&c, 120);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return c.accum_err;
}

static int samsung_ea8074_enable(struct drm_panel *panel)
{
	struct samsung_ea8074 *ctx = to_samsung_ea8074(panel);
	struct mipi_dsi_multi_context c = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_on_multi(&c);
	mipi_dsi_msleep(&c, 20);

	mipi_dsi_dcs_write_seq_multi(&c, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&c, 0xb0, 0x05);
	mipi_dsi_dcs_write_seq_multi(&c, 0xb1, 0x40);
	mipi_dsi_dcs_write_seq_multi(&c, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&c, 0xb6, 0xa2);
	mipi_dsi_dcs_write_seq_multi(&c, 0xf0, 0xa5, 0xa5);

	return c.accum_err;
}

static int samsung_ea8074_disable(struct drm_panel *panel)
{
	struct samsung_ea8074 *ctx = to_samsung_ea8074(panel);
	struct mipi_dsi_multi_context c = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&c);
	mipi_dsi_usleep_range(&c, 5000, 10000);

	return c.accum_err;
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
	.type = DRM_MODE_TYPE_DRIVER,
};

static int samsung_ea8074_get_modes(struct drm_panel *panel,
				    struct drm_connector *connector)
{
	connector->display_info.bpc = 8;
	return drm_connector_helper_get_modes_fixed(connector,
						    &samsung_ea8074_mode);
}

static enum drm_panel_orientation
samsung_ea8074_get_orientation(struct drm_panel *panel)
{
	struct samsung_ea8074 *ctx = to_samsung_ea8074(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs samsung_ea8074_panel_funcs = {
	.prepare = samsung_ea8074_prepare,
	.unprepare = samsung_ea8074_unprepare,
	.enable = samsung_ea8074_enable,
	.disable = samsung_ea8074_disable,
	.get_modes = samsung_ea8074_get_modes,
	.get_orientation = samsung_ea8074_get_orientation,
};

static int samsung_ea8074_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static const struct backlight_ops samsung_ea8074_bl_ops = {
	.update_status = samsung_ea8074_bl_update_status,
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

	ctx = devm_drm_panel_alloc(dev, struct samsung_ea8074, panel,
				   &samsung_ea8074_panel_funcs,
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

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = samsung_ea8074_create_backlight(dsi);
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
	{ /* sentinel */ }
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

MODULE_AUTHOR("Robin Reckmann <robin.reckmann@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Samsung EA8074 panel");
MODULE_LICENSE("GPL");
