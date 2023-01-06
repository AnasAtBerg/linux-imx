/*
 * Copyright (C) 2022 Avnet Embedded GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>

enum dsi_cmd_type {
	INIT_DCS_CMD,
	DELAY_CMD,
};

struct panel_init_cmd {
	enum dsi_cmd_type type;
	size_t len;
	const char *data;
};

struct panel_desc {
	const struct drm_display_mode *display_mode;
	unsigned int bpc;
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
	const struct panel_init_cmd *init_cmds;
};

struct panel_info {
	struct drm_panel base;
	struct mipi_dsi_device *link;
	const struct panel_desc *desc;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	unsigned int reset_delay;
	unsigned int reset_duration;
	bool prepared;
	bool enabled;
};

#define _INIT_DCS_CMD(...) { \
	.type = INIT_DCS_CMD, \
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} \
	}

#define _INIT_DELAY_CMD(...) { \
	.type = DELAY_CMD,\
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} \
	}

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, base);
}

static int __dsi_gen_write(struct mipi_dsi_device *dsi,
		const void *buffer, size_t size)
{
	int ret = mipi_dsi_generic_write(dsi, buffer, size);
	usleep_range(200, 250);
	return ret;
}

static int send_mipi_cmds(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	const struct panel_init_cmd *init_cmds = pinfo->desc->init_cmds;
	unsigned int i = 0;
	int ret;

	for (i = 0; init_cmds[i].len != 0; i++) {
		const struct panel_init_cmd *cmd = &init_cmds[i];

		switch (cmd->type) {
		case DELAY_CMD:
			msleep(cmd->data[0]);
			ret = 0;
			break;
		case INIT_DCS_CMD:
#if defined(__DEBUG)
			{
				char prefix[8];
				snprintf(prefix, 8, "%2d : ", i);
				print_hex_dump(KERN_INFO, prefix, DUMP_PREFIX_NONE, 20, 1,
						&cmd->data[0], cmd->len, false);
			}
#endif /* defined(__DEBUG) */
			ret = __dsi_gen_write(pinfo->link, &cmd->data[0], cmd->len);
			break;

		default:
			ret = -EINVAL;
		}

		if (ret < 0) {
			dev_err(panel->dev, "failed to write command %u\n", i);
			return ret;
		}
	}

	return 0;
}

static int panel_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (!pinfo->enabled)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(pinfo->link);
	if (ret < 0) {
		dev_err(panel->dev, "failed to set display off: %d\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->link);
	if (ret < 0) {
		dev_err(panel->dev, "failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	pinfo->enabled = false;

	return 0;
}

static int panel_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	if (!pinfo->prepared)
		return 0;

	gpiod_set_value(pinfo->reset_gpio, 0);

	pinfo->prepared = false;

	return 0;
}

static int panel_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	if (pinfo->prepared)
		return 0;

	gpiod_set_value(pinfo->reset_gpio, 1);
	usleep_range(pinfo->reset_delay, pinfo->reset_delay + 5000);

	gpiod_set_value(pinfo->reset_gpio, 0);
	usleep_range(pinfo->reset_duration, pinfo->reset_duration + 5000);

	gpiod_set_value(pinfo->reset_gpio, 1);
	usleep_range(pinfo->reset_delay, pinfo->reset_delay + 5000);

	pinfo->prepared = true;

	return 0;
}

static int panel_enable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (pinfo->enabled)
		return 0;

	/* send init code */
	ret = send_mipi_cmds(panel);
	if (ret < 0) {
		dev_err(panel->dev, "failed to send DCS Init Code: %d\n", ret);
		goto poweroff;
	}

	pinfo->enabled = true;

	return 0;

poweroff:
	gpiod_set_value(pinfo->reset_gpio, 0);
	return ret;
}

static int panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	const struct drm_display_mode *m = pinfo->desc->display_mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, m);
	if (!mode) {
		dev_err(pinfo->base.dev, "failed to add mode %ux%u@%u\n",
			m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;

	return 1;
}

static const struct drm_panel_funcs panel_funcs = {
	.disable = panel_disable,
	.unprepare = panel_unprepare,
	.prepare = panel_prepare,
	.enable = panel_enable,
	.get_modes = panel_get_modes,
};

static const struct drm_display_mode default_display_mode = {
	.clock = 47000,
	.hdisplay = 400,
	.hsync_start = 400 + 50,
	.hsync_end = 400 + 50 + 50,
	.htotal = 400 + 50 + 50 + 30,
	.vdisplay = 1280,
	.vsync_start = 1280 + 30,
	.vsync_end = 1280 + 30 + 30,
	.vtotal = 1280 + 30 + 30 + 20,
};

static const struct panel_init_cmd xinli_x078dtlt_119_init_cmds[] = {
	_INIT_DELAY_CMD(150),
	_INIT_DCS_CMD(0xB0, 0x5A),
	_INIT_DCS_CMD(0xB1, 0x00),
	_INIT_DCS_CMD(0x89, 0x01),
	_INIT_DCS_CMD(0x91, 0x17),
	_INIT_DCS_CMD(0xB1, 0x03),
	_INIT_DCS_CMD(0x2C, 0x28),
	_INIT_DCS_CMD(0x00, 0xB7),
	_INIT_DCS_CMD(0x01, 0x1B),
	_INIT_DCS_CMD(0x02, 0x00),
	_INIT_DCS_CMD(0x03, 0x00),
	_INIT_DCS_CMD(0x04, 0x00),
	_INIT_DCS_CMD(0x05, 0x00),
	_INIT_DCS_CMD(0x06, 0x00),
	_INIT_DCS_CMD(0x07, 0x00),
	_INIT_DCS_CMD(0x08, 0x00),
	_INIT_DCS_CMD(0x09, 0x00),
	_INIT_DCS_CMD(0x0A, 0x01),
	_INIT_DCS_CMD(0x0B, 0x01),
	_INIT_DCS_CMD(0x0C, 0x20),
	_INIT_DCS_CMD(0x0D, 0x00),
	_INIT_DCS_CMD(0x0E, 0x24),
	_INIT_DCS_CMD(0x0F, 0x1C),
	_INIT_DCS_CMD(0x10, 0xC9),
	_INIT_DCS_CMD(0x11, 0x60),
	_INIT_DCS_CMD(0x12, 0x70),
	_INIT_DCS_CMD(0x13, 0x01),
	_INIT_DCS_CMD(0x14, 0xE3),
	_INIT_DCS_CMD(0x15, 0xFF),
	_INIT_DCS_CMD(0x16, 0x3D),
	_INIT_DCS_CMD(0x17, 0x0E),
	_INIT_DCS_CMD(0x18, 0x01),
	_INIT_DCS_CMD(0x19, 0x00),
	_INIT_DCS_CMD(0x1A, 0x00),
	_INIT_DCS_CMD(0x1B, 0xFC),
	_INIT_DCS_CMD(0x1C, 0x0B),
	_INIT_DCS_CMD(0x1D, 0xA0),
	_INIT_DCS_CMD(0x1E, 0x03),
	_INIT_DCS_CMD(0x1F, 0x04),
	_INIT_DCS_CMD(0x20, 0x0C),
	_INIT_DCS_CMD(0x21, 0x00),
	_INIT_DCS_CMD(0x22, 0x04),
	_INIT_DCS_CMD(0x23, 0x81),
	_INIT_DCS_CMD(0x24, 0x1F),
	_INIT_DCS_CMD(0x25, 0x10),
	_INIT_DCS_CMD(0x26, 0x9B),
	_INIT_DCS_CMD(0x2D, 0x01),
	_INIT_DCS_CMD(0x2E, 0x84),
	_INIT_DCS_CMD(0x2F, 0x00),
	_INIT_DCS_CMD(0x30, 0x02),
	_INIT_DCS_CMD(0x31, 0x08),
	_INIT_DCS_CMD(0x32, 0x01),
	_INIT_DCS_CMD(0x33, 0x1C),
	_INIT_DCS_CMD(0x34, 0x70),
	_INIT_DCS_CMD(0x35, 0xFF),
	_INIT_DCS_CMD(0x36, 0xFF),
	_INIT_DCS_CMD(0x37, 0xFF),
	_INIT_DCS_CMD(0x38, 0xFF),
	_INIT_DCS_CMD(0x39, 0xFF),
	_INIT_DCS_CMD(0x3A, 0x05),
	_INIT_DCS_CMD(0x3B, 0x00),
	_INIT_DCS_CMD(0x3C, 0x00),
	_INIT_DCS_CMD(0x3D, 0x00),
	_INIT_DCS_CMD(0x3E, 0xCF),
	_INIT_DCS_CMD(0x3F, 0x84),
	_INIT_DCS_CMD(0x40, 0x28),
	_INIT_DCS_CMD(0x41, 0xFC),
	_INIT_DCS_CMD(0x42, 0x01),
	_INIT_DCS_CMD(0x43, 0x40),
	_INIT_DCS_CMD(0x44, 0x05),
	_INIT_DCS_CMD(0x45, 0xE8),
	_INIT_DCS_CMD(0x46, 0x16),
	_INIT_DCS_CMD(0x47, 0x00),
	_INIT_DCS_CMD(0x48, 0x00),
	_INIT_DCS_CMD(0x49, 0x88),
	_INIT_DCS_CMD(0x4A, 0x08),
	_INIT_DCS_CMD(0x4B, 0x05),
	_INIT_DCS_CMD(0x4C, 0x03),
	_INIT_DCS_CMD(0x4D, 0xD0),
	_INIT_DCS_CMD(0x4E, 0x13),
	_INIT_DCS_CMD(0x4F, 0xFF),
	_INIT_DCS_CMD(0x50, 0x0A),
	_INIT_DCS_CMD(0x51, 0x53),
	_INIT_DCS_CMD(0x52, 0x26),
	_INIT_DCS_CMD(0x53, 0x22),
	_INIT_DCS_CMD(0x54, 0x09),
	_INIT_DCS_CMD(0x55, 0x22),
	_INIT_DCS_CMD(0x56, 0x00),
	_INIT_DCS_CMD(0x57, 0x1C),
	_INIT_DCS_CMD(0x58, 0x03),
	_INIT_DCS_CMD(0x59, 0x3F),
	_INIT_DCS_CMD(0x5A, 0x28),
	_INIT_DCS_CMD(0x5B, 0x01),
	_INIT_DCS_CMD(0x5C, 0xCC),
	_INIT_DCS_CMD(0x5D, 0x21),
	_INIT_DCS_CMD(0x5E, 0x84),
	_INIT_DCS_CMD(0x5F, 0x10),
	_INIT_DCS_CMD(0x60, 0x42),
	_INIT_DCS_CMD(0x61, 0x40),
	_INIT_DCS_CMD(0x62, 0x06),
	_INIT_DCS_CMD(0x63, 0x3A),
	_INIT_DCS_CMD(0x64, 0xA6),
	_INIT_DCS_CMD(0x65, 0x04),
	_INIT_DCS_CMD(0x66, 0x09),
	_INIT_DCS_CMD(0x67, 0x21),
	_INIT_DCS_CMD(0x68, 0x84),
	_INIT_DCS_CMD(0x69, 0x10),
	_INIT_DCS_CMD(0x6A, 0x42),
	_INIT_DCS_CMD(0x6B, 0x08),
	_INIT_DCS_CMD(0x6C, 0x21),
	_INIT_DCS_CMD(0x6D, 0x84),
	_INIT_DCS_CMD(0x6E, 0x74),
	_INIT_DCS_CMD(0x6F, 0xE2),
	_INIT_DCS_CMD(0x70, 0x6B),
	_INIT_DCS_CMD(0x71, 0x6B),
	_INIT_DCS_CMD(0x72, 0x94),
	_INIT_DCS_CMD(0x73, 0x10),
	_INIT_DCS_CMD(0x74, 0x42),
	_INIT_DCS_CMD(0x75, 0x08),
	_INIT_DCS_CMD(0x76, 0x00),
	_INIT_DCS_CMD(0x77, 0x00),
	_INIT_DCS_CMD(0x78, 0x0F),
	_INIT_DCS_CMD(0x79, 0xE0),
	_INIT_DCS_CMD(0x7A, 0x01),
	_INIT_DCS_CMD(0x7B, 0xFF),
	_INIT_DCS_CMD(0x7C, 0xFF),
	_INIT_DCS_CMD(0x7D, 0x0F),
	_INIT_DCS_CMD(0x7E, 0x41),
	_INIT_DCS_CMD(0x7F, 0xFE),
	_INIT_DCS_CMD(0xB1, 0x02),
	_INIT_DCS_CMD(0x00, 0xFF),
	_INIT_DCS_CMD(0x01, 0x05),
	_INIT_DCS_CMD(0x02, 0xC8),
	_INIT_DCS_CMD(0x03, 0x00),
	_INIT_DCS_CMD(0x04, 0x14),
	_INIT_DCS_CMD(0x05, 0x4B),
	_INIT_DCS_CMD(0x06, 0x64),
	_INIT_DCS_CMD(0x07, 0x0A),
	_INIT_DCS_CMD(0x08, 0xC0),
	_INIT_DCS_CMD(0x09, 0x00),
	_INIT_DCS_CMD(0x0A, 0x00),
	_INIT_DCS_CMD(0x0B, 0x10),
	_INIT_DCS_CMD(0x0C, 0xE6),
	_INIT_DCS_CMD(0x0D, 0x0D),
	_INIT_DCS_CMD(0x0F, 0x00),
	_INIT_DCS_CMD(0x10, 0x79),
	_INIT_DCS_CMD(0x11, 0xAB),
	_INIT_DCS_CMD(0x12, 0xA7),
	_INIT_DCS_CMD(0x13, 0xD7),
	_INIT_DCS_CMD(0x14, 0x7B),
	_INIT_DCS_CMD(0x15, 0x9D),
	_INIT_DCS_CMD(0x16, 0x74),
	_INIT_DCS_CMD(0x17, 0x6D),
	_INIT_DCS_CMD(0x18, 0x73),
	_INIT_DCS_CMD(0x19, 0xB3),
	_INIT_DCS_CMD(0x1A, 0x6E),
	_INIT_DCS_CMD(0x1B, 0x0E),
	_INIT_DCS_CMD(0x1C, 0xFF),
	_INIT_DCS_CMD(0x1D, 0xFF),
	_INIT_DCS_CMD(0x1E, 0xFF),
	_INIT_DCS_CMD(0x1F, 0xFF),
	_INIT_DCS_CMD(0x20, 0xFF),
	_INIT_DCS_CMD(0x21, 0xFF),
	_INIT_DCS_CMD(0x22, 0xFF),
	_INIT_DCS_CMD(0x23, 0xFF),
	_INIT_DCS_CMD(0x24, 0xFF),
	_INIT_DCS_CMD(0x25, 0xFF),
	_INIT_DCS_CMD(0x26, 0xFF),
	_INIT_DCS_CMD(0x27, 0x1F),
	_INIT_DCS_CMD(0x28, 0xFF),
	_INIT_DCS_CMD(0x29, 0xFF),
	_INIT_DCS_CMD(0x2A, 0xFF),
	_INIT_DCS_CMD(0x2B, 0xFF),
	_INIT_DCS_CMD(0x2C, 0xFF),
	_INIT_DCS_CMD(0x2D, 0x07),
	_INIT_DCS_CMD(0x33, 0x3F),
	_INIT_DCS_CMD(0x35, 0x7F),
	_INIT_DCS_CMD(0x36, 0x3F),
	_INIT_DCS_CMD(0x38, 0xFF),
	_INIT_DCS_CMD(0x3A, 0x80),
	_INIT_DCS_CMD(0x3B, 0x01),
	_INIT_DCS_CMD(0x3C, 0x80),
	_INIT_DCS_CMD(0x3D, 0x2C),
	_INIT_DCS_CMD(0x3E, 0x00),
	_INIT_DCS_CMD(0x3F, 0x90),
	_INIT_DCS_CMD(0x40, 0x05),
	_INIT_DCS_CMD(0x41, 0x00),
	_INIT_DCS_CMD(0x42, 0xB2),
	_INIT_DCS_CMD(0x43, 0x00),
	_INIT_DCS_CMD(0x44, 0x40),
	_INIT_DCS_CMD(0x45, 0x06),
	_INIT_DCS_CMD(0x46, 0x00),
	_INIT_DCS_CMD(0x47, 0x00),
	_INIT_DCS_CMD(0x48, 0x9B),
	_INIT_DCS_CMD(0x49, 0xD2),
	_INIT_DCS_CMD(0x4A, 0x21),
	_INIT_DCS_CMD(0x4B, 0x43),
	_INIT_DCS_CMD(0x4C, 0x16),
	_INIT_DCS_CMD(0x4D, 0xC0),
	_INIT_DCS_CMD(0x4E, 0x0F),
	_INIT_DCS_CMD(0x4F, 0xF1),
	_INIT_DCS_CMD(0x50, 0x78),
	_INIT_DCS_CMD(0x51, 0x7A),
	_INIT_DCS_CMD(0x52, 0x34),
	_INIT_DCS_CMD(0x53, 0x99),
	_INIT_DCS_CMD(0x54, 0xA2),
	_INIT_DCS_CMD(0x55, 0x02),
	_INIT_DCS_CMD(0x56, 0x14),
	_INIT_DCS_CMD(0x57, 0xB8),
	_INIT_DCS_CMD(0x58, 0xDC),
	_INIT_DCS_CMD(0x59, 0xD4),
	_INIT_DCS_CMD(0x5A, 0xEF),
	_INIT_DCS_CMD(0x5B, 0xF7),
	_INIT_DCS_CMD(0x5C, 0xFB),
	_INIT_DCS_CMD(0x5D, 0xFD),
	_INIT_DCS_CMD(0x5E, 0x7E),
	_INIT_DCS_CMD(0x5F, 0xBF),
	_INIT_DCS_CMD(0x60, 0xEF),
	_INIT_DCS_CMD(0x61, 0xE6),
	_INIT_DCS_CMD(0x62, 0x76),
	_INIT_DCS_CMD(0x63, 0x73),
	_INIT_DCS_CMD(0x64, 0xBB),
	_INIT_DCS_CMD(0x65, 0xDD),
	_INIT_DCS_CMD(0x66, 0x6E),
	_INIT_DCS_CMD(0x67, 0x37),
	_INIT_DCS_CMD(0x68, 0x8C),
	_INIT_DCS_CMD(0x69, 0x08),
	_INIT_DCS_CMD(0x6A, 0x31),
	_INIT_DCS_CMD(0x6B, 0xB8),
	_INIT_DCS_CMD(0x6C, 0xB8),
	_INIT_DCS_CMD(0x6D, 0xB8),
	_INIT_DCS_CMD(0x6E, 0xB8),
	_INIT_DCS_CMD(0x6F, 0xB8),
	_INIT_DCS_CMD(0x70, 0x5C),
	_INIT_DCS_CMD(0x71, 0x2E),
	_INIT_DCS_CMD(0x72, 0x17),
	_INIT_DCS_CMD(0x73, 0x00),
	_INIT_DCS_CMD(0x74, 0x00),
	_INIT_DCS_CMD(0x75, 0x00),
	_INIT_DCS_CMD(0x76, 0x00),
	_INIT_DCS_CMD(0x77, 0x00),
	_INIT_DCS_CMD(0x78, 0x00),
	_INIT_DCS_CMD(0x79, 0x00),
	_INIT_DCS_CMD(0x7A, 0xDC),
	_INIT_DCS_CMD(0x7B, 0xDC),
	_INIT_DCS_CMD(0x7C, 0xDC),
	_INIT_DCS_CMD(0x7D, 0xDC),
	_INIT_DCS_CMD(0x7E, 0xDC),
	_INIT_DCS_CMD(0x7F, 0x6E),
	_INIT_DCS_CMD(0x0B, 0x00),
	_INIT_DCS_CMD(0xB1, 0x03),
	_INIT_DCS_CMD(0x2C, 0x2C),
	_INIT_DCS_CMD(0xB1, 0x00),
	_INIT_DCS_CMD(0x89, 0x03),
	_INIT_DELAY_CMD(150),
	{},
};

static const struct panel_desc xinli_x078dtlt_119_panel_desc = {
	.display_mode = &default_display_mode,
	.bpc = 8,
	.width_mm = 59,
	.height_mm = 190,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_CLOCK_NON_CONTINUOUS |
		MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
	.init_cmds = xinli_x078dtlt_119_init_cmds,
};

static const struct of_device_id panel_of_match[] = {
	{
		.compatible = "xinli,x078dtlt-119",
		.data = &xinli_x078dtlt_119_panel_desc,
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, panel_of_match);

static int panel_add(struct panel_info *pinfo)
{
	struct device *dev = &pinfo->link->dev;
	struct device_node *np = dev->of_node;
	int ret;

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio)) {
		ret = PTR_ERR(pinfo->reset_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get reset gpio: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "reset-delay-us", &pinfo->reset_delay);
	if ( ret != 0 ) {
		pinfo->reset_delay = 5000;
	}
	dev_info(dev, "using reset_delay of %u us\n", pinfo->reset_delay);

	ret = of_property_read_u32(np, "reset_duration_us", &pinfo->reset_duration);
	if ( ret != 0 ) {
		pinfo->reset_duration = 20000;
	}
	dev_info(dev, "using reset_duration of %u us\n", pinfo->reset_duration);

	pinfo->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->enable_gpio)) {
		ret = PTR_ERR(pinfo->enable_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get enable gpio: %d\n", ret);
		return ret;
	}

	gpiod_set_value(pinfo->reset_gpio, 0);
	gpiod_set_value(pinfo->enable_gpio, 1);

	drm_panel_init(&pinfo->base, dev, &panel_funcs,
			DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&pinfo->base);
	if (ret)
		return ret;

	drm_panel_add(&pinfo->base);

	return 0;
}

static int panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct panel_info *pinfo;
	const struct panel_desc *desc;
	struct device_node *np = dev->of_node;
	int ret;

	pinfo = devm_kzalloc(dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->mode_flags;

	if (of_property_read_bool(np, "mipi-dsi.continuous-clock")) {
		dsi->mode_flags &= ~(MIPI_DSI_CLOCK_NON_CONTINUOUS);
		dev_info(dev, "using continuous-clock\n");
	}


	dsi->format = desc->format;
	dsi->lanes = desc->lanes;
	pinfo->desc = desc;

	pinfo->link = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	ret = panel_add(pinfo);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&pinfo->base);

	return ret;
}

static int panel_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = panel_disable(&pinfo->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = panel_unprepare(&pinfo->base);
	if (ret)
		dev_err(&dsi->dev, "failed to unprepare panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&pinfo->base);

	return 0;
}

static void panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);

	panel_disable(&pinfo->base);
	panel_unprepare(&pinfo->base);
}

static struct mipi_dsi_driver panel_driver = {
	.driver = {
		.name = "panel-xinli-x078dtlt-119",
		.of_match_table = panel_of_match,
	},
	.probe = panel_probe,
	.remove = panel_remove,
	.shutdown = panel_shutdown,
};
module_mipi_dsi_driver(panel_driver);

MODULE_DESCRIPTION("XINLI X078DTLT-119 driver");
MODULE_LICENSE("GPL v2");
