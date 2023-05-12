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
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/media-bus-format.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
//diz #include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <video/display_timing.h>
#include <video/videomode.h>
#include <video/of_display_timing.h>

struct xinli_tcxd101iblma_panel_desc {
	const struct drm_display_mode *modes;
	struct display_timing *timings;
	unsigned int bpc;

	/**
	 * @width:width (in mm) of the panel's active display area
	 * @height: height(in mm) of the panel's active display area
	 */
	struct {
		unsigned int width;
		unsigned int height;
	} size;

	u32 bus_format;
	u32 bus_flags;
};


struct xinli_tcxd101iblma_panel {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;
	struct xinli_tcxd101iblma_panel_desc *desc;

	struct gpio_desc	*enable_gpio;
	struct gpio_desc	*reset_gpio;
};

enum xinli_tcxd101iblma_op {
	OP_DELAY,
	OP_SWITCH_PAGE,
	OP_COMMAND,
};

struct xinli_tcxd101iblma_instr {
	enum xinli_tcxd101iblma_op	op;

	union arg {
		struct cmd {
			u8	cmd;
			u8	data;
		} cmd;
		u8	page;
	} arg;
};

#define _INIT_SWITCH_PAGE_INSTR(_page)	\
	{					\
		.op = OP_SWITCH_PAGE,	\
		.arg = {			\
			.page = (_page),	\
		},				\
	}

#define _INIT_COMMAND_INSTR(_cmd, _data)		\
	{						\
		.op = OP_COMMAND,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (_data),	\
			},				\
		},					\
	}

#define _INIT_DELAY_CMD(_delay) { \
		.op = OP_DELAY,	\
		.arg = {			\
			.page = (_delay),	\
		},				\
	}

static const struct xinli_tcxd101iblma_instr xinli_tcxd101iblma_init[] = {
	_INIT_SWITCH_PAGE_INSTR(0x03),
	//GIP_1
	_INIT_COMMAND_INSTR(0x01,0x00),
	_INIT_COMMAND_INSTR(0x02,0x00),
	_INIT_COMMAND_INSTR(0x03,0x53),        //STVA=STV2_4
	_INIT_COMMAND_INSTR(0x04,0xD3),        //STVB=STV1_3
	_INIT_COMMAND_INSTR(0x05,0x00),
	_INIT_COMMAND_INSTR(0x06,0x0D),        //STVA_Rise
	_INIT_COMMAND_INSTR(0x07,0x08),        //STVB_Rise
	_INIT_COMMAND_INSTR(0x08,0x00),
	_INIT_COMMAND_INSTR(0x09,0x00),
	_INIT_COMMAND_INSTR(0x0a,0x00),
	_INIT_COMMAND_INSTR(0x0b,0x00),
	_INIT_COMMAND_INSTR(0x0c,0x00),
	_INIT_COMMAND_INSTR(0x0d,0x00),
	_INIT_COMMAND_INSTR(0x0e,0x00),
	_INIT_COMMAND_INSTR(0x0f,0x28),        //CLW1(ALR) Duty=45%
	_INIT_COMMAND_INSTR(0x10,0x28),        //CLW2(ARR) Duty=45%
	_INIT_COMMAND_INSTR(0x11,0x00),
	_INIT_COMMAND_INSTR(0x12,0x00),
	_INIT_COMMAND_INSTR(0x13,0x00),        //CLWX(ATF)
	_INIT_COMMAND_INSTR(0x14,0x00),
	_INIT_COMMAND_INSTR(0x15,0x00),
	_INIT_COMMAND_INSTR(0x16,0x00),
	_INIT_COMMAND_INSTR(0x17,0x00),
	_INIT_COMMAND_INSTR(0x18,0x00),
	_INIT_COMMAND_INSTR(0x19,0x00),
	_INIT_COMMAND_INSTR(0x1a,0x00),
	_INIT_COMMAND_INSTR(0x1b,0x00),
	_INIT_COMMAND_INSTR(0x1c,0x00),
	_INIT_COMMAND_INSTR(0x1d,0x00),
	_INIT_COMMAND_INSTR(0x1e,0x40),        //CLKA
	_INIT_COMMAND_INSTR(0x1f,0x80),
	_INIT_COMMAND_INSTR(0x20,0x06),        //CLKA_Rise
	_INIT_COMMAND_INSTR(0x21,0x01),        //CLKA_Fall
	_INIT_COMMAND_INSTR(0x22,0x00),
	_INIT_COMMAND_INSTR(0x23,0x00),
	_INIT_COMMAND_INSTR(0x24,0x00),
	_INIT_COMMAND_INSTR(0x25,0x00),
	_INIT_COMMAND_INSTR(0x26,0x00),
	_INIT_COMMAND_INSTR(0x27,0x00),
	_INIT_COMMAND_INSTR(0x28,0x33),       //CLK Phase
	_INIT_COMMAND_INSTR(0x29,0x33),       //CLK overlap
	_INIT_COMMAND_INSTR(0x2a,0x00),
	_INIT_COMMAND_INSTR(0x2b,0x00),
	_INIT_COMMAND_INSTR(0x2c,0x00),
	_INIT_COMMAND_INSTR(0x2d,0x00),
	_INIT_COMMAND_INSTR(0x2e,0x00),
	_INIT_COMMAND_INSTR(0x2f,0x00),
	_INIT_COMMAND_INSTR(0x30,0x00),
	_INIT_COMMAND_INSTR(0x32,0x00),
	_INIT_COMMAND_INSTR(0x33,0x00),
	_INIT_COMMAND_INSTR(0x34,0x03),     //VDD1&2 overlap 0us
	_INIT_COMMAND_INSTR(0x35,0x00),
	_INIT_COMMAND_INSTR(0x36,0x00),
	_INIT_COMMAND_INSTR(0x37,0x00),
	_INIT_COMMAND_INSTR(0x38,0x96),     //VDD1&2 toggle 2.5sec
	_INIT_COMMAND_INSTR(0x39,0x00),
	_INIT_COMMAND_INSTR(0x3a,0x00),
	_INIT_COMMAND_INSTR(0x3b,0x00),
	_INIT_COMMAND_INSTR(0x3c,0x00),
	_INIT_COMMAND_INSTR(0x3d,0x00),
	_INIT_COMMAND_INSTR(0x3e,0x00),
	_INIT_COMMAND_INSTR(0x3f,0x00),
	_INIT_COMMAND_INSTR(0x40,0x00),
	_INIT_COMMAND_INSTR(0x41,0x00),
	_INIT_COMMAND_INSTR(0x42,0x00),
	_INIT_COMMAND_INSTR(0x43,0x00),
	_INIT_COMMAND_INSTR(0x44,0x00),
	//GIP_2
	_INIT_COMMAND_INSTR(0x50,0x00),
	_INIT_COMMAND_INSTR(0x51,0x23),
	_INIT_COMMAND_INSTR(0x52,0x45),
	_INIT_COMMAND_INSTR(0x53,0x67),
	_INIT_COMMAND_INSTR(0x54,0x89),
	_INIT_COMMAND_INSTR(0x55,0xAB),
	_INIT_COMMAND_INSTR(0x56,0x01),
	_INIT_COMMAND_INSTR(0x57,0x23),
	_INIT_COMMAND_INSTR(0x58,0x45),
	_INIT_COMMAND_INSTR(0x59,0x67),
	_INIT_COMMAND_INSTR(0x5a,0x89),
	_INIT_COMMAND_INSTR(0x5b,0xAB),
	_INIT_COMMAND_INSTR(0x5c,0xCD),
	_INIT_COMMAND_INSTR(0x5d,0xEF),
	//GIP_3
	_INIT_COMMAND_INSTR(0x5e,0x00),
	_INIT_COMMAND_INSTR(0x5f,0x08),     //FW_CGOUT_L[1]    STV3
	_INIT_COMMAND_INSTR(0x60,0x08),     //FW_CGOUT_L[2]    STV3
	_INIT_COMMAND_INSTR(0x61,0x06),     //FW_CGOUT_L[3]    STV4
	_INIT_COMMAND_INSTR(0x62,0x06),     //FW_CGOUT_L[4]    STV4
	_INIT_COMMAND_INSTR(0x63,0x01),     //FW_CGOUT_L[5]    VDS
	_INIT_COMMAND_INSTR(0x64,0x01),     //FW_CGOUT_L[6]    VDS
	_INIT_COMMAND_INSTR(0x65,0x00),     //FW_CGOUT_L[7]    VSD
	_INIT_COMMAND_INSTR(0x66,0x00),     //FW_CGOUT_L[8]    VSD
	_INIT_COMMAND_INSTR(0x67,0x02),     //FW_CGOUT_L[9]
	_INIT_COMMAND_INSTR(0x68,0x15),     //FW_CGOUT_L[10]   VDD2
	_INIT_COMMAND_INSTR(0x69,0x15),     //FW_CGOUT_L[11]   VDD2
	_INIT_COMMAND_INSTR(0x6a,0x14),     //FW_CGOUT_L[12]   VDD1
	_INIT_COMMAND_INSTR(0x6b,0x14),     //FW_CGOUT_L[13]   VDD1
	_INIT_COMMAND_INSTR(0x6c,0x0D),     //FW_CGOUT_L[14]   CLK8
	_INIT_COMMAND_INSTR(0x6d,0x0D),     //FW_CGOUT_L[15]   CLK8
	_INIT_COMMAND_INSTR(0x6e,0x0C),     //FW_CGOUT_L[16]   CLK6
	_INIT_COMMAND_INSTR(0x6f,0x0C),     //FW_CGOUT_L[17]   CLK6
	_INIT_COMMAND_INSTR(0x70,0x0F),     //FW_CGOUT_L[18]   CLK4
	_INIT_COMMAND_INSTR(0x71,0x0F),     //FW_CGOUT_L[19]   CLK4
	_INIT_COMMAND_INSTR(0x72,0x0E),     //FW_CGOUT_L[20]   CLK2
	_INIT_COMMAND_INSTR(0x73,0x0E),     //FW_CGOUT_L[21]   CLK2
	_INIT_COMMAND_INSTR(0x74,0x02),     //FW_CGOUT_L[22]   VGL
	_INIT_COMMAND_INSTR(0x75,0x08),     //BW_CGOUT_L[1]
	_INIT_COMMAND_INSTR(0x76,0x08),     //BW_CGOUT_L[2]
	_INIT_COMMAND_INSTR(0x77,0x06),     //BW_CGOUT_L[3]
	_INIT_COMMAND_INSTR(0x78,0x06),     //BW_CGOUT_L[4]
	_INIT_COMMAND_INSTR(0x79,0x01),     //BW_CGOUT_L[5]
	_INIT_COMMAND_INSTR(0x7a,0x01),     //BW_CGOUT_L[6]
	_INIT_COMMAND_INSTR(0x7b,0x00),     //BW_CGOUT_L[7]
	_INIT_COMMAND_INSTR(0x7c,0x00),     //BW_CGOUT_L[8]
	_INIT_COMMAND_INSTR(0x7d,0x02),     //BW_CGOUT_L[9]
	_INIT_COMMAND_INSTR(0x7e,0x15),     //BW_CGOUT_L[10]
	_INIT_COMMAND_INSTR(0x7f,0x15),     //BW_CGOUT_L[11]
	_INIT_COMMAND_INSTR(0x80,0x14),     //BW_CGOUT_L[12]
	_INIT_COMMAND_INSTR(0x81,0x14),     //BW_CGOUT_L[13]
	_INIT_COMMAND_INSTR(0x82,0x0D),     //BW_CGOUT_L[14]
	_INIT_COMMAND_INSTR(0x83,0x0D),     //BW_CGOUT_L[15]
	_INIT_COMMAND_INSTR(0x84,0x0C),     //BW_CGOUT_L[16]
	_INIT_COMMAND_INSTR(0x85,0x0C),     //BW_CGOUT_L[17]
	_INIT_COMMAND_INSTR(0x86,0x0F),     //BW_CGOUT_L[18]
	_INIT_COMMAND_INSTR(0x87,0x0F),     //BW_CGOUT_L[19]
	_INIT_COMMAND_INSTR(0x88,0x0E),     //BW_CGOUT_L[20]
	_INIT_COMMAND_INSTR(0x89,0x0E),     //BW_CGOUT_L[21]
	_INIT_COMMAND_INSTR(0x8A,0x02),     //BW_CGOUT_L[22]
	//CMD_Page 4
	_INIT_SWITCH_PAGE_INSTR(0x04),
	_INIT_COMMAND_INSTR(0x6E,0x2B),           //VGH 15V
	_INIT_COMMAND_INSTR(0x6F,0x37),           //reg
	_INIT_COMMAND_INSTR(0x3A,0xA4),           //POWER SAVING
	_INIT_COMMAND_INSTR(0x8D,0x1A),           //VGL -11V
	_INIT_COMMAND_INSTR(0x87,0xBA),           //ESD
	_INIT_COMMAND_INSTR(0xB2,0xD1),
	_INIT_COMMAND_INSTR(0x88,0x0B),
	_INIT_COMMAND_INSTR(0x38,0x01),
	_INIT_COMMAND_INSTR(0x39,0x00),
	_INIT_COMMAND_INSTR(0xB5,0x07),
	_INIT_COMMAND_INSTR(0x31,0x75),
	_INIT_COMMAND_INSTR(0x3B,0x98),
	//CMD_Page 1
	_INIT_SWITCH_PAGE_INSTR(0x01),
	_INIT_COMMAND_INSTR(0x22,0x0A),          //BGR), W_D(0x SS
	_INIT_COMMAND_INSTR(0x31,0x00),          //Column inversion
	_INIT_COMMAND_INSTR(0x53,0x40),          //VCOM1
	_INIT_COMMAND_INSTR(0x55,0x40),          //VCOM2
	_INIT_COMMAND_INSTR(0x50,0x99),          //VREG1OUT 4.5V
	_INIT_COMMAND_INSTR(0x51,0x94),          //VREG2OUT -4.5V
	_INIT_COMMAND_INSTR(0x60,0x10),         //SDT 3.5us
	_INIT_COMMAND_INSTR(0x62,0x20),
	//============Gamma START=============
	//Pos Register
	_INIT_COMMAND_INSTR(0xA0,0x00),
	_INIT_COMMAND_INSTR(0xA1,0x00),
	_INIT_COMMAND_INSTR(0xA2,0x15),
	_INIT_COMMAND_INSTR(0xA3,0x14),
	_INIT_COMMAND_INSTR(0xA4,0x1B),
	_INIT_COMMAND_INSTR(0xA5,0x2F),
	_INIT_COMMAND_INSTR(0xA6,0x25),
	_INIT_COMMAND_INSTR(0xA7,0x24),
	_INIT_COMMAND_INSTR(0xA8,0x80),
	_INIT_COMMAND_INSTR(0xA9,0x1F),
	_INIT_COMMAND_INSTR(0xAA,0x2C),
	_INIT_COMMAND_INSTR(0xAB,0x6C),
	_INIT_COMMAND_INSTR(0xAC,0x16),
	_INIT_COMMAND_INSTR(0xAD,0x14),
	_INIT_COMMAND_INSTR(0xAE,0x4D),
	_INIT_COMMAND_INSTR(0xAF,0x20),
	_INIT_COMMAND_INSTR(0xB0,0x29),
	_INIT_COMMAND_INSTR(0xB1,0x4F),
	_INIT_COMMAND_INSTR(0xB2,0x5F),
	_INIT_COMMAND_INSTR(0xB3,0x23),
	//Neg Register
	_INIT_COMMAND_INSTR(0xC0,0x00),
	_INIT_COMMAND_INSTR(0xC1,0x2E),
	_INIT_COMMAND_INSTR(0xC2,0x3B),
	_INIT_COMMAND_INSTR(0xC3,0x15),
	_INIT_COMMAND_INSTR(0xC4,0x16),
	_INIT_COMMAND_INSTR(0xC5,0x28),
	_INIT_COMMAND_INSTR(0xC6,0x1A),
	_INIT_COMMAND_INSTR(0xC7,0x1C),
	_INIT_COMMAND_INSTR(0xC8,0xA7),
	_INIT_COMMAND_INSTR(0xC9,0x1B),
	_INIT_COMMAND_INSTR(0xCA,0x28),
	_INIT_COMMAND_INSTR(0xCB,0x92),
	_INIT_COMMAND_INSTR(0xCC,0x1F),
	_INIT_COMMAND_INSTR(0xCD,0x1C),
	_INIT_COMMAND_INSTR(0xCE,0x4B),
	_INIT_COMMAND_INSTR(0xCF,0x1F),
	_INIT_COMMAND_INSTR(0xD0,0x28),
	_INIT_COMMAND_INSTR(0xD1,0x4E),
	_INIT_COMMAND_INSTR(0xD2,0x5C),
	_INIT_COMMAND_INSTR(0xD3,0x23),
	//============ Gamma END===========
	//CMD_Page 0
	_INIT_SWITCH_PAGE_INSTR(0x00),
	_INIT_COMMAND_INSTR(0x11,0x00),
	_INIT_DELAY_CMD(120),
	_INIT_COMMAND_INSTR(0x29,0x00),
	_INIT_COMMAND_INSTR(0x35,0x00),
};

static inline struct xinli_tcxd101iblma_panel *panel_to_xinli_tcxd101iblma(struct drm_panel *panel)
{
	return container_of(panel, struct xinli_tcxd101iblma_panel, panel);
}

static int xinli_tcxd101iblma_switch_page(struct xinli_tcxd101iblma_panel *tftcp, u8 page)
{
	u8 buf[4] = { 0xff, 0x98, 0x81, page };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(tftcp->dsi, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&tftcp->dsi->dev, "Failed to switch_page[%d] (%d)\n", page, ret);
		return ret;
	}

	return 0;
}

static int xinli_tcxd101iblma_send_cmd_data(struct xinli_tcxd101iblma_panel *tftcp, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(tftcp->dsi, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&tftcp->dsi->dev, "Failed to send_cmd_data[%02x,%02X] (%d)\n", cmd, data, ret);
		return ret;
	}

	return 0;
}

static void xinli_tcxd101iblma_reset(struct xinli_tcxd101iblma_panel *tftcp)
{
	/* Reset 5ms */
	if (tftcp->reset_gpio) {
		gpiod_set_value_cansleep(tftcp->reset_gpio, 0);
		usleep_range(5000, 10000);
		gpiod_set_value_cansleep(tftcp->reset_gpio, 1);
		usleep_range(20000, 25000);
	}
}

static int xinli_tcxd101iblma_prepare(struct drm_panel *panel)
{
	struct xinli_tcxd101iblma_panel *tftcp = panel_to_xinli_tcxd101iblma(panel);

	dev_info(&tftcp->dsi->dev,"%s\n",__func__);

    xinli_tcxd101iblma_reset(tftcp);

	return 0;
}

static int xinli_tcxd101iblma_enable(struct drm_panel *panel)
{
	struct xinli_tcxd101iblma_panel *tftcp = panel_to_xinli_tcxd101iblma(panel);
	int ret;
	unsigned int i;

	tftcp->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	for (i = 0; i < ARRAY_SIZE(xinli_tcxd101iblma_init); i++) {
		const struct xinli_tcxd101iblma_instr *instr = &xinli_tcxd101iblma_init[i];

		if (instr->op == OP_DELAY) {
			msleep(instr->arg.page);
			ret = 0;
		}else if (instr->op == OP_SWITCH_PAGE)
			ret = xinli_tcxd101iblma_switch_page(tftcp, instr->arg.page);
		else if (instr->op == OP_COMMAND)
			ret = xinli_tcxd101iblma_send_cmd_data(tftcp, instr->arg.cmd.cmd,
						      instr->arg.cmd.data);
		if (ret)
			return ret;
	}

	ret = xinli_tcxd101iblma_switch_page(tftcp, 0);
	if (ret)
		return ret;

    /* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(tftcp->dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(&tftcp->dsi->dev, "Failed to set tear ON (%d)\n", ret);
		return ret;
	}

	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(tftcp->dsi);
	if (ret < 0) {
		dev_err(&tftcp->dsi->dev, "Failed to exit sleep mode (%d)\n", ret);
		return ret;
	}

	usleep_range(120000, 130000);

	ret = mipi_dsi_dcs_set_display_on(tftcp->dsi);
	if (ret < 0) {
		dev_err(&tftcp->dsi->dev, "Failed to set display ON (%d)\n", ret);
		return ret;
	}

	dev_info(&tftcp->dsi->dev,"%s\n",__func__);

	dev_info(panel->dev, "EN success ...\n");

	return 0;
}

static int xinli_tcxd101iblma_disable(struct drm_panel *panel)
{
	struct xinli_tcxd101iblma_panel *tftcp = panel_to_xinli_tcxd101iblma(panel);
	int ret;

	tftcp->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(tftcp->dsi);
	if (ret < 0) {
		dev_err(&tftcp->dsi->dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(100000, 110000);

	ret = mipi_dsi_dcs_enter_sleep_mode(tftcp->dsi);
	if (ret < 0) {
		dev_err(&tftcp->dsi->dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int xinli_tcxd101iblma_unprepare(struct drm_panel *panel)
{
	struct xinli_tcxd101iblma_panel *tftcp = panel_to_xinli_tcxd101iblma(panel);

	dev_info(&tftcp->dsi->dev,"%s\n",__func__);

	if (tftcp->enable_gpio != NULL)
		gpiod_set_value_cansleep(tftcp->enable_gpio, 0);
	if (tftcp->reset_gpio != NULL)
		gpiod_set_value_cansleep(tftcp->reset_gpio, 0);

	return 0;
}

static int xinli_tcxd101iblma_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	struct xinli_tcxd101iblma_panel *tftcp = panel_to_xinli_tcxd101iblma(panel);
	struct drm_display_mode *mode;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	struct videomode vm;

	if (!tftcp->desc)
		return 0;

	videomode_from_timing(tftcp->desc->timings, &vm);
	mode = drm_mode_create(connector->dev);
	if (!mode) {
		dev_err(tftcp->panel.dev, "failed to add mode \n");
	}

	drm_display_mode_from_videomode(&vm, mode);
	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;;

	dev_info(tftcp->panel.dev, "%d %d %d.....\n",
	tftcp->desc->size.width, tftcp->desc->size.height, tftcp->desc->bpc);

	connector->display_info.bpc = tftcp->desc->bpc;
	connector->display_info.width_mm = tftcp->desc->size.width;
	connector->display_info.height_mm = tftcp->desc->size.height;

	drm_display_info_set_bus_formats(&connector->display_info,
									&bus_format, 1);
	connector->display_info.bus_flags = tftcp->desc->bus_flags;

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs xinli_tcxd101iblma_funcs = {
	.prepare	= xinli_tcxd101iblma_prepare,
	.unprepare	= xinli_tcxd101iblma_unprepare,
	.enable		= xinli_tcxd101iblma_enable,
	.disable	= xinli_tcxd101iblma_disable,
	.get_modes	= xinli_tcxd101iblma_get_modes,
};

static int xinli_tcxd101iblma_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct xinli_tcxd101iblma_panel *tftcp;
	struct display_timing *timing;
	int ret;
	unsigned int w, h;

	tftcp = devm_kzalloc(&dsi->dev, sizeof(*tftcp), GFP_KERNEL);
	if (!tftcp)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, tftcp);
	tftcp->dsi = dsi;

	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	/* non-burst mode with sync pulse */
	dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	dsi->format = MIPI_DSI_FMT_RGB888;

	dev_info(&dsi->dev, "\nprobing...\n");

	tftcp->reset_gpio = devm_gpiod_get_optional(&dsi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(tftcp->reset_gpio))
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");

	if (tftcp->reset_gpio != NULL) {
                gpiod_set_value_cansleep(tftcp->reset_gpio,1);
        }
	tftcp->enable_gpio = devm_gpiod_get_optional(&dsi->dev, "enable", GPIOD_OUT_HIGH);
        if (IS_ERR(tftcp->enable_gpio))
                dev_err(&dsi->dev, "Couldn't get our enable GPIO\n");

	if (tftcp->enable_gpio != NULL) {
		gpiod_set_value_cansleep(tftcp->enable_gpio,1);
	}

	ret = of_property_read_u32(dsi->dev.of_node, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(&dsi->dev, "Failed to get dsi-lanes property, use default setting\n");
		dsi->lanes = 4;
	}

	tftcp->desc = devm_kzalloc(&dsi->dev, sizeof(tftcp->desc), GFP_KERNEL);
	if (!tftcp->desc)
		return -ENOMEM;

	timing = devm_kzalloc(&dsi->dev, sizeof(*timing), GFP_KERNEL);
	if (!timing)
		return -ENOMEM;

	ret = of_get_display_timing(dsi->dev.of_node, "display-timings", timing);
	if (ret) {
		dev_err(&dsi->dev, "failed to get display timing\n");
		return ret;
	}

	tftcp->desc->timings = timing;
	of_property_read_u32(dsi->dev.of_node, "width", &w);
	of_property_read_u32(dsi->dev.of_node, "height", &h);
	tftcp->desc->size.width = w;
	tftcp->desc->size.height = h;
	tftcp->desc->bpc = 8;

	dev_info(&dsi->dev, "drm_panel_init...\n");

	drm_panel_init(&tftcp->panel, &dsi->dev, &xinli_tcxd101iblma_funcs,
			DRM_MODE_CONNECTOR_DSI);
	ret = drm_panel_of_backlight(&tftcp->panel);
        if (ret)
                return ret;

	drm_panel_add(&tftcp->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&tftcp->panel);

	dev_set_drvdata(&dsi->dev, tftcp);

	dev_info(&dsi->dev, "probe success ...\n");

	return ret;
}

static int xinli_tcxd101iblma_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct xinli_tcxd101iblma_panel *tftcp = mipi_dsi_get_drvdata(dsi);

	xinli_tcxd101iblma_disable(&tftcp->panel);
	mipi_dsi_detach(dsi);

	if (tftcp->panel.dev)
		drm_panel_remove(&tftcp->panel);

	return 0;
}

static void xinli_tcxd101iblma_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	struct xinli_tcxd101iblma_panel *tftcp = mipi_dsi_get_drvdata(dsi);

	xinli_tcxd101iblma_disable(&tftcp->panel);
	xinli_tcxd101iblma_unprepare(&tftcp->panel);
}

static const struct of_device_id xinli_tcxd101iblma_of_match[] = {
	{
		.compatible = "xinli,tcxd101iblma",
	},{
	}
};
MODULE_DEVICE_TABLE(of, xinli_tcxd101iblma_of_match);

static struct mipi_dsi_driver xinli_tcxd101iblma_dsi_driver = {
	.probe		= xinli_tcxd101iblma_dsi_probe,
	.remove		= xinli_tcxd101iblma_dsi_remove,
	.shutdown	= xinli_tcxd101iblma_dsi_shutdown,
	.driver = {
		.name		= "panel-xinli-tcxd101iblma",
		.of_match_table	= xinli_tcxd101iblma_of_match,
	},
};
module_mipi_dsi_driver(xinli_tcxd101iblma_dsi_driver);

MODULE_AUTHOR("Dizni Premdas <Dizni.Premdas@avnet.eu>");
MODULE_DESCRIPTION("XINLI TCXD101IBLMA Driver");
MODULE_LICENSE("GPL v2");
