/*
 * Copyright (C) 2018 CopuLab Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_print.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include "sn65dsi83.h"

/* Register addresses */
#define SN65DSI83_SOFT_RESET			0x09
#define SN65DSI83_CORE_PLL				0x0A
	#define LVDS_CLK_RANGE_SHIFT		1
	#define HS_CLK_SRC_SHIFT			0

#define SN65DSI83_PLL_DIV				0x0B
	#define DSI_CLK_DIV_SHIFT			3

#define SN65DSI83_PLL_EN				0x0D
#define SN65DSI83_DSI_CFG				0x10
	#define CHA_DSI_LANES_SHIFT			3

#define SN65DSI83_DSI_EQ				0x11
#define SN65DSI83_CHA_DSI_CLK_RNG		0x12
#define SN65DSI83_CHB_DSI_CLK_RNG		0x13
#define SN65DSI83_LVDS_MODE				0x18
	#define DE_NEG_POLARITY_SHIFT		7
	#define HS_NEG_POLARITY_SHIFT		6
	#define VS_NEG_POLARITY_SHIFT		5
	#define LVDS_LINK_CFG_SHIFT			4
	#define CHA_24BPP_MODE_SHIFT		3
	#define CHB_24BPP_MODE_SHIFT		2
	#define CHA_24BPP_FMT1_SHIFT		1
	#define CHB_24BPP_FMT1_SHIFT		0

#define SN65DSI83_LVDS_SIGN				0x19
#define SN65DSI83_LVDS_TERM				0x1A
#define SN65DSI83_LVDS_CM_ADJ			0x1B
#define SN65DSI83_CHA_LINE_LEN_LO		0x20
#define SN65DSI83_CHA_LINE_LEN_HI		0x21
#define SN65DSI83_CHB_LINE_LEN_LO		0x22
#define SN65DSI83_CHB_LINE_LEN_HI		0x23
#define SN65DSI83_CHA_VERT_LINES_LO		0x24
#define SN65DSI83_CHA_VERT_LINES_HI		0x25
#define SN65DSI83_CHB_VERT_LINES_LO		0x26
#define SN65DSI83_CHB_VERT_LINES_HI		0x27
#define SN65DSI83_CHA_SYNC_DELAY_LO		0x28
#define SN65DSI83_CHA_SYNC_DELAY_HI		0x29
#define SN65DSI83_CHB_SYNC_DELAY_LO		0x2A
#define SN65DSI83_CHB_SYNC_DELAY_HI		0x2B
#define SN65DSI83_CHA_HSYNC_WIDTH_LO	0x2C
#define SN65DSI83_CHA_HSYNC_WIDTH_HI	0x2D
#define SN65DSI83_CHB_HSYNC_WIDTH_LO	0x2E
#define SN65DSI83_CHB_HSYNC_WIDTH_HI	0x2F
#define SN65DSI83_CHA_VSYNC_WIDTH_LO	0x30
#define SN65DSI83_CHA_VSYNC_WIDTH_HI	0x31
#define SN65DSI83_CHB_VSYNC_WIDTH_LO	0x32
#define SN65DSI83_CHB_VSYNC_WIDTH_HI	0x33
#define SN65DSI83_CHA_HORZ_BACKPORCH	0x34
#define SN65DSI83_CHB_HORZ_BACKPORCH	0x35
#define SN65DSI83_CHA_VERT_BACKPORCH	0x36
#define SN65DSI83_CHB_VERT_BACKPORCH	0x37
#define SN65DSI83_CHA_HORZ_FRONTPORCH	0x38
#define SN65DSI83_CHB_HORZ_FRONTPORCH	0x39
#define SN65DSI83_CHA_VERT_FRONTPORCH	0x3A
#define SN65DSI83_CHB_VERT_FRONTPORCH	0x3B
#define SN65DSI83_TEST_PATTERN			0x3C
#define SN65DSI83_REG_3D				0x3D
#define SN65DSI83_REG_3E				0x3E
#define SN65DSI83_REG_E0				0xE0
#define SN65DSI83_REG_E1				0xE1
#define SN65DSI83_CHA_ERR				0xE5


static int sn65dsi83_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret) {
		dev_err(&client->dev, "failed to write at 0x%02x", reg);
		return ret;
	}

	dev_dbg(&client->dev, "%s: write reg 0x%02x data 0x%02x", __func__, reg, val);

	return ret;
}
#define SN65DSI83_WRITE(reg,val) sn65dsi83_write(client, (reg) , (val))

static int sn65dsi83_read(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x", reg);
		return ret;
	}

	dev_dbg(&client->dev, "%s: read reg 0x%02x data 0x%02x", __func__, reg, ret);

	return ret;
}
#define SN65DSI83_READ(reg) sn65dsi83_read(client, (reg))

static const unsigned char registers_to_show[] = {
	SN65DSI83_SOFT_RESET,
	SN65DSI83_CORE_PLL,
	SN65DSI83_PLL_DIV,
	SN65DSI83_PLL_EN,
	SN65DSI83_DSI_CFG,
	SN65DSI83_DSI_EQ,
	SN65DSI83_CHA_DSI_CLK_RNG,
	SN65DSI83_CHB_DSI_CLK_RNG,
	SN65DSI83_LVDS_MODE,
	SN65DSI83_LVDS_SIGN,
	SN65DSI83_LVDS_TERM,
	SN65DSI83_LVDS_CM_ADJ,
	SN65DSI83_CHA_LINE_LEN_LO,
	SN65DSI83_CHA_LINE_LEN_HI,
	SN65DSI83_CHB_LINE_LEN_LO,
	SN65DSI83_CHB_LINE_LEN_HI,
	SN65DSI83_CHA_VERT_LINES_LO,
	SN65DSI83_CHA_VERT_LINES_HI,
	SN65DSI83_CHB_VERT_LINES_LO,
	SN65DSI83_CHB_VERT_LINES_HI,
	SN65DSI83_CHA_SYNC_DELAY_LO,
	SN65DSI83_CHA_SYNC_DELAY_HI,
	SN65DSI83_CHB_SYNC_DELAY_LO,
	SN65DSI83_CHB_SYNC_DELAY_HI,
	SN65DSI83_CHA_HSYNC_WIDTH_LO,
	SN65DSI83_CHA_HSYNC_WIDTH_HI,
	SN65DSI83_CHB_HSYNC_WIDTH_LO,
	SN65DSI83_CHB_HSYNC_WIDTH_HI,
	SN65DSI83_CHA_VSYNC_WIDTH_LO,
	SN65DSI83_CHA_VSYNC_WIDTH_HI,
	SN65DSI83_CHB_VSYNC_WIDTH_LO,
	SN65DSI83_CHB_VSYNC_WIDTH_HI,
	SN65DSI83_CHA_HORZ_BACKPORCH,
	SN65DSI83_CHB_HORZ_BACKPORCH,
	SN65DSI83_CHA_VERT_BACKPORCH,
	SN65DSI83_CHB_VERT_BACKPORCH,
	SN65DSI83_CHA_HORZ_FRONTPORCH,
	SN65DSI83_CHB_HORZ_FRONTPORCH,
	SN65DSI83_CHA_VERT_FRONTPORCH,
	SN65DSI83_CHB_VERT_FRONTPORCH,
	SN65DSI83_TEST_PATTERN,
	SN65DSI83_REG_3D,
	SN65DSI83_REG_3E,
	SN65DSI83_REG_E0,
	SN65DSI83_REG_E1,
	SN65DSI83_CHA_ERR,
};

static ssize_t sn65dsi83_reg_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct sn65dsi83 *sn = dev_get_drvdata(dev);
	struct sn65dsi83_brg *brg = sn->brg;
	struct i2c_client *i2c = brg->client;
	int offset = 0;
	int reg, val, idx;

	offset += sprintf(&buf[offset], "%4s | %4s \n", "reg", "val");
	offset += sprintf(&buf[offset], "----------- \n");
	for (idx = 0; idx<ARRAY_SIZE(registers_to_show); idx++) {
		reg = registers_to_show[idx];
		val = sn65dsi83_read(i2c, reg);
		offset += sprintf(&buf[offset], "0x%02x | 0x%02x\n", reg, val);
	}

	sn65dsi83_write(i2c, SN65DSI83_CHA_ERR, 0xff);

	return offset;
}

static DEVICE_ATTR(sn65dsi83_reg, 0444, sn65dsi83_reg_show, NULL);

static ssize_t sn65dsi83_stat_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct sn65dsi83 *sn = dev_get_drvdata(dev);
	struct sn65dsi83_brg *brg = sn->brg;
	int offset = 0;

	offset += sprintf(&buf[offset], "Width .................... %d [mm]\n",
			brg->width_mm);
	offset += sprintf(&buf[offset], "Height ................... %d [mm]\n",
			brg->height_mm);
	offset += sprintf(&buf[offset], "Format ................... 0x%x\n",
			brg->format);
	offset += sprintf(&buf[offset], "Bpp ...................... %d\n",
			brg->bpp);
	offset += sprintf(&buf[offset], "Nunber of DSI lanes ...... %d\n",
			brg->num_dsi_lanes);
	offset += sprintf(&buf[offset], "Nunber of LVDS chanels ... %d\n",
			brg->num_channels);
	offset += sprintf(&buf[offset], "Negative DE polarity ..... %s\n",
			brg->de_neg_polarity ? "enabled" : "disabled");
	offset += sprintf(&buf[offset], "Burst mode ............... %s\n",
			brg->burst_mode ? "enabled" : "disabled");
	offset += sprintf(&buf[offset], "Test pattern generator ... %s\n",
			brg->test_pattern ? "enabled" : "disabled");
	offset += sprintf(&buf[offset], "\nVideo mode:\n");
	offset += sprintf(&buf[offset], "  Pixel clock ............ %ld [Hz]\n",
			brg->vm.pixelclock);
	offset += sprintf(&buf[offset], "  Horizontal:\n");
	offset += sprintf(&buf[offset], "    Active ............... %d\n",
			brg->vm.hactive);
	offset += sprintf(&buf[offset], "    Front porch .......... %d\n",
			brg->vm.hfront_porch);
	offset += sprintf(&buf[offset], "    Back porch ........... %d\n",
			brg->vm.hback_porch);
	offset += sprintf(&buf[offset], "    Sync len ............. %d\n",
			brg->vm.hsync_len);
	offset += sprintf(&buf[offset], "  Vertical:\n");
	offset += sprintf(&buf[offset], "    Active ............... %d\n",
			brg->vm.vactive);
	offset += sprintf(&buf[offset], "    Front porch .......... %d\n",
			brg->vm.vfront_porch);
	offset += sprintf(&buf[offset], "    Back porch ........... %d\n",
			brg->vm.vback_porch);
	offset += sprintf(&buf[offset], "    Sync len ............. %d\n",
			brg->vm.vsync_len);
	offset += sprintf(&buf[offset], "  Flags .................. 0x%x\n",
			brg->vm.flags);

	return offset;
}

static DEVICE_ATTR(sn65dsi83_stat, 0444, sn65dsi83_stat_show, NULL);


static int sn65dsi83_brg_power_on(struct sn65dsi83_brg *brg)
{
	dev_dbg(&brg->client->dev,"%s\n",__func__);

	if (brg->gpio_enable) {
		gpiod_set_value_cansleep(brg->gpio_enable, 1);
		/* Wait for 1ms for the internal voltage regulator to stabilize */
		msleep(1);
	}

	return 0;
}

static void sn65dsi83_brg_power_off(struct sn65dsi83_brg *brg)
{
	dev_dbg(&brg->client->dev,"%s\n",__func__);

	if (brg->gpio_enable) {
		gpiod_set_value_cansleep(brg->gpio_enable, 0);
		/*
		 * The EN pin must be held low for at least 10 ms
		 * before being asserted high
		 */
		msleep(10);
	}
}

static int sn65dsi83_brg_start_stream(struct sn65dsi83_brg *brg)
{
	struct i2c_client *client = I2C_CLIENT(brg);

	dev_dbg(&client->dev,"%s\n",__func__);

	/* Set the PLL_EN bit (CSR 0x0D.0) */
	SN65DSI83_WRITE(SN65DSI83_PLL_EN, 0x1);

	/* Wait for the PLL_LOCK bit to be set (CSR 0x0A.7) */
	msleep(200);

	/* Perform SW reset to apply changes */
	SN65DSI83_WRITE(SN65DSI83_SOFT_RESET, 0x01);

	SN65DSI83_WRITE(SN65DSI83_CHA_ERR, 0xff);
	SN65DSI83_WRITE(SN65DSI83_REG_E1, 0x74);
	SN65DSI83_WRITE(SN65DSI83_REG_E0, 0x01);

	dev_info(&client->dev, "Error/lock (E5): 0x%02x\n",
			SN65DSI83_READ(SN65DSI83_CHA_ERR));

	if (brg->gpio_panel_enable)
		gpiod_set_value_cansleep(brg->gpio_panel_enable, 1);

	return 0;
}

static void sn65dsi83_brg_stop_stream(struct sn65dsi83_brg *brg)
{
	struct i2c_client *client = I2C_CLIENT(brg);
	dev_dbg(&client->dev,"%s\n",__func__);
	/* Clear the PLL_EN bit (CSR 0x0D.0) */
	SN65DSI83_WRITE(SN65DSI83_PLL_EN, 0x00);

	if (brg->gpio_panel_enable)
		gpiod_set_value_cansleep(brg->gpio_panel_enable, 0);
}

static int sn65dsi83_calk_clk_range(int min_regval, int max_regval,
		unsigned long min_clk, unsigned long inc, unsigned long target_clk)
{
	int regval = min_regval;
	unsigned long clk = min_clk;

	while (regval <= max_regval) {
		if ((clk <= target_clk) && (target_clk < (clk + inc)))
			return regval;

		regval++;
		clk += inc;
	}

	return -1;
}

#define ABS(X) ((X) < 0 ? (-1 * (X)) : (X))
static int sn65dsi83_calk_div(int min_regval, int max_regval, int min_div,
		int inc, unsigned long source_clk, unsigned long target_clk)
{
	int regval = min_regval;
	int div = min_div;
	unsigned long curr_delta;
	unsigned long prev_delta = ABS(DIV_ROUND_UP(source_clk, div) -
					target_clk);

	while (regval <= max_regval) {
		curr_delta = ABS(DIV_ROUND_UP(source_clk, div) - target_clk);
		if (curr_delta > prev_delta)
			return --regval;

		regval++;
		div += inc;
	}

	return -1;
}

static int sn65dsi83_brg_configure(struct sn65dsi83_brg *brg)
{
	int regval = 0;
	struct i2c_client *client = I2C_CLIENT(brg);
	struct videomode *vm = VM(brg);

	u32 dsi_clk = (((PIXCLK * BPP(brg)) / DSI_LANES(brg)) >> 1);
	bool dual_channel_lvds = (brg->num_channels == 2);
	u32 lvds_clk = dual_channel_lvds ? (PIXCLK >> 1) : PIXCLK;
	u32 cha_hsync_width = dual_channel_lvds ? (HPW >> 1) : HPW;
	u32 cha_horz_backporch = dual_channel_lvds ? (HBP >> 1) : HBP;

	dev_info(&client->dev, "Resolution:      %d x %d\n", HACTIVE, VACTIVE);
	dev_info(&client->dev, "BPP:             %d\n", BPP(brg));
	dev_info(&client->dev, "DSI lanes:       %d\n", DSI_LANES(brg));
	dev_info(&client->dev, "LVDS channels:   %d\n", dual_channel_lvds ? 2 : 1);
	dev_info(&client->dev, "DSI clock:       %u Hz\n", dsi_clk);

	/* Reset PLL_EN and SOFT_RESET registers */
	SN65DSI83_WRITE(SN65DSI83_SOFT_RESET,0x00);
	SN65DSI83_WRITE(SN65DSI83_PLL_EN,0x00);

	/* LVDS clock setup */
	if ((25000000 <= lvds_clk) && (lvds_clk < 37500000))
		regval = 0;
	else
		regval = sn65dsi83_calk_clk_range(0x01, 0x05, 37500000, 25000000,
					lvds_clk);

	dev_info(&client->dev, "LVDS clock:      %u Hz\n", lvds_clk);
	dev_dbg(&client->dev, "lvds_clk_range = 0x%x\n", regval);

	if (regval < 0) {
		dev_err(&client->dev, "failed to configure LVDS clock");
		return -EINVAL;
	}

	regval = (regval << LVDS_CLK_RANGE_SHIFT);
	regval |= (1 << HS_CLK_SRC_SHIFT); /* Use DSI clock */
	SN65DSI83_WRITE(SN65DSI83_CORE_PLL,regval);

	/* DSI clock range */
	regval = sn65dsi83_calk_clk_range(0x08, 0x64, 40000000, 5000000, dsi_clk);
	if (regval < 0) {
		dev_err(&client->dev, "failed to configure DSI clock range\n");
		return -EINVAL;
	}
	SN65DSI83_WRITE(SN65DSI83_CHA_DSI_CLK_RNG,regval);
	SN65DSI83_WRITE(SN65DSI83_CHB_DSI_CLK_RNG,0x00);

	dev_dbg(&client->dev, "dsi_clk_range = 0x%x\n", regval);

	/* DSI clock divider */
	regval = sn65dsi83_calk_div(0x0, 0x18, 1, 1, dsi_clk, lvds_clk);
	if (regval < 0) {
		dev_err(&client->dev, "failed to calculate DSI clock divider");
		return -EINVAL;
	}

	dev_dbg(&client->dev, "dsi_clk_div = 0x%x\n", regval);

	regval = regval << DSI_CLK_DIV_SHIFT;
	SN65DSI83_WRITE(SN65DSI83_PLL_DIV,regval);

	/* Configure DSI_LANES  */
	regval = SN65DSI83_READ(SN65DSI83_DSI_CFG);
	regval &= ~(3 << CHA_DSI_LANES_SHIFT);
	regval |= ((4 - DSI_LANES(brg)) << CHA_DSI_LANES_SHIFT);
	SN65DSI83_WRITE(SN65DSI83_DSI_CFG,regval);

	/* CHA_DSI_DATA_EQ - No Equalization */
	/* CHA_DSI_CLK_EQ  - No Equalization */
	SN65DSI83_WRITE(SN65DSI83_DSI_EQ,0x00);

	/* Video formats */
	regval = 0;
	if (FLAGS & DISPLAY_FLAGS_HSYNC_LOW)
		regval |= (1 << HS_NEG_POLARITY_SHIFT);

	if (FLAGS & DISPLAY_FLAGS_VSYNC_LOW)
		regval |= (1 << VS_NEG_POLARITY_SHIFT);

	if (brg->de_neg_polarity)
		regval |= (1 << DE_NEG_POLARITY_SHIFT);

	if (BPP(brg) == 24) {
		regval |= (1 << CHA_24BPP_MODE_SHIFT);
		regval |= (1 << CHB_24BPP_MODE_SHIFT);
	}

	if (FORMAT(brg) == 1) {
		regval |= (1 << CHA_24BPP_FMT1_SHIFT);
		regval |= (1 << CHB_24BPP_FMT1_SHIFT);
	}

	regval |= ((dual_channel_lvds ? 0 : 1) << LVDS_LINK_CFG_SHIFT);
	SN65DSI83_WRITE(SN65DSI83_LVDS_MODE,regval);

	/* Voltage and pins */
	SN65DSI83_WRITE(SN65DSI83_LVDS_SIGN,0x00);
	SN65DSI83_WRITE(SN65DSI83_LVDS_TERM,0x03);
	SN65DSI83_WRITE(SN65DSI83_LVDS_CM_ADJ,0x00);

	/* Configure sync delay to minimal allowed value */
	SN65DSI83_WRITE(SN65DSI83_CHA_SYNC_DELAY_LO,0x21);
	SN65DSI83_WRITE(SN65DSI83_CHA_SYNC_DELAY_HI,0x00);

	SN65DSI83_WRITE(SN65DSI83_CHB_SYNC_DELAY_LO,0x00);
	SN65DSI83_WRITE(SN65DSI83_CHB_SYNC_DELAY_HI,0x00);

	/* Geometry */
	SN65DSI83_WRITE(SN65DSI83_CHA_LINE_LEN_LO,LOW(HACTIVE));
	SN65DSI83_WRITE(SN65DSI83_CHA_LINE_LEN_HI,HIGH(HACTIVE));

	SN65DSI83_WRITE(SN65DSI83_CHB_LINE_LEN_LO,0x00);
	SN65DSI83_WRITE(SN65DSI83_CHB_LINE_LEN_HI,0x00);

	SN65DSI83_WRITE(SN65DSI83_CHA_VERT_LINES_LO,LOW(VACTIVE));
	SN65DSI83_WRITE(SN65DSI83_CHA_VERT_LINES_HI,HIGH(VACTIVE));

	SN65DSI83_WRITE(SN65DSI83_CHB_VERT_LINES_LO,0x00);
	SN65DSI83_WRITE(SN65DSI83_CHB_VERT_LINES_HI,0x00);

	SN65DSI83_WRITE(SN65DSI83_CHA_HSYNC_WIDTH_LO,LOW(cha_hsync_width));
	SN65DSI83_WRITE(SN65DSI83_CHA_HSYNC_WIDTH_HI,HIGH(HPW));

	SN65DSI83_WRITE(SN65DSI83_CHB_HSYNC_WIDTH_LO,0x00);
	SN65DSI83_WRITE(SN65DSI83_CHB_HSYNC_WIDTH_HI,0x00);

	SN65DSI83_WRITE(SN65DSI83_CHA_VSYNC_WIDTH_LO,LOW(VPW));
	SN65DSI83_WRITE(SN65DSI83_CHA_VSYNC_WIDTH_HI,HIGH(VPW));

	SN65DSI83_WRITE(SN65DSI83_CHB_VSYNC_WIDTH_LO,0x00);
	SN65DSI83_WRITE(SN65DSI83_CHB_VSYNC_WIDTH_HI,0x00);

	SN65DSI83_WRITE(SN65DSI83_CHA_HORZ_BACKPORCH,LOW(cha_horz_backporch));
	SN65DSI83_WRITE(SN65DSI83_CHA_VERT_BACKPORCH,LOW(VBP));

	SN65DSI83_WRITE(SN65DSI83_CHB_HORZ_BACKPORCH,0x00);
	SN65DSI83_WRITE(SN65DSI83_CHB_VERT_BACKPORCH,0x00);

	SN65DSI83_WRITE(SN65DSI83_CHA_HORZ_FRONTPORCH,LOW(HFP));
	SN65DSI83_WRITE(SN65DSI83_CHA_VERT_FRONTPORCH,LOW(VFP));

	SN65DSI83_WRITE(SN65DSI83_CHB_HORZ_FRONTPORCH,0x00);
	SN65DSI83_WRITE(SN65DSI83_CHB_VERT_FRONTPORCH,0x00);

	SN65DSI83_WRITE(SN65DSI83_TEST_PATTERN,0x00);
	SN65DSI83_WRITE(SN65DSI83_REG_3D,0x00);
	SN65DSI83_WRITE(SN65DSI83_REG_3E,0x00);

	SN65DSI83_WRITE(SN65DSI83_TEST_PATTERN, brg->test_pattern ? (1<<4) : 0);

	{
		int reg = SN65DSI83_CHA_ERR;
		int val = sn65dsi83_read(client, reg);
		dev_dbg(&client->dev, "Error/lock (E5) = 0x%02x\n", val);
	}

	return 0;
}

static int sn65dsi83_brg_setup(struct sn65dsi83_brg *brg)
{
	struct i2c_client *client = I2C_CLIENT(brg);
	dev_dbg(&client->dev,"%s\n",__func__);
	sn65dsi83_brg_power_on(brg);
	return sn65dsi83_brg_configure(brg);
}

static int sn65dsi83_brg_reset(struct sn65dsi83_brg *brg)
{
	/* Soft Reset reg value at power on should be 0x00 */
	struct i2c_client *client = I2C_CLIENT(brg);
	int ret = SN65DSI83_READ(SN65DSI83_SOFT_RESET);
	dev_dbg(&client->dev,"%s\n",__func__);
	if (ret != 0x00) {
		dev_err(&client->dev,"Failed to reset the device");
		return -ENODEV;
	}
	return 0;
}

static struct sn65dsi83_brg_funcs brg_func = {
	.power_on = sn65dsi83_brg_power_on,
	.power_off = sn65dsi83_brg_power_off,
	.setup = sn65dsi83_brg_setup,
	.reset = sn65dsi83_brg_reset,
	.start_stream = sn65dsi83_brg_start_stream,
	.stop_stream = sn65dsi83_brg_stop_stream,
};

static struct sn65dsi83_brg brg = {
		.funcs = &brg_func,
};

struct sn65dsi83_brg *sn65dsi83_brg_get(void) {
	return &brg;
}

int sn65dsi83_brg_probe(struct sn65dsi83_brg *brg)
{
	struct i2c_client *i2c = brg->client;
	struct device *dev = &i2c->dev;
	int ret;

	ret = device_create_file(dev, &dev_attr_sn65dsi83_reg);
	if (ret < 0)
		dev_warn(dev, "failed to add sn65dsi83 bridge reg sysfs\n");
	ret = device_create_file(dev, &dev_attr_sn65dsi83_stat);
	if (ret < 0)
		dev_warn(dev, "failed to add sn65dsi83 bridge stat sysfs\n");

	return 0;
}

void sn65dsi83_brg_remove(struct sn65dsi83_brg *brg)
{
	struct i2c_client *i2c = brg->client;
	struct device *dev = &i2c->dev;

	device_remove_file(dev, &dev_attr_sn65dsi83_reg);
	device_remove_file(dev, &dev_attr_sn65dsi83_stat);
}
