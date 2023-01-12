// SPDX-License-Identifier: GPL-2.0-only
/*
 * MFD core driver for Ricoh RN5T618 PMIC
 *
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
 * Copyright (C) 2016 Toradex AG
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rn5t618.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <dt-bindings/mfd/ricoh-rn5t618.h>

static const struct mfd_cell rn5t618_cells[] = {
	{ .name = "rn5t618-regulator" },
	{ .name = "rn5t618-wdt" },
};

static const struct mfd_cell rc5t619_cells[] = {
	{ .name = "rn5t618-adc" },
	{ .name = "rn5t618-power" },
	{ .name = "rn5t618-regulator" },
	{ .name = "rc5t619-rtc" },
	{ .name = "rn5t618-wdt" },
};

static bool rn5t618_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RN5T618_WATCHDOGCNT:
	case RN5T618_DCIRQ:
	case RN5T618_ILIMDATAH ... RN5T618_AIN0DATAL:
	case RN5T618_ADCCNT3:
	case RN5T618_IR_ADC1 ... RN5T618_IR_ADC3:
	case RN5T618_IR_GPR:
	case RN5T618_IR_GPF:
	case RN5T618_MON_IOIN:
	case RN5T618_INTMON:
	case RN5T618_RTC_CTRL1 ... RN5T618_RTC_CTRL2:
	case RN5T618_RTC_SECONDS ... RN5T618_RTC_YEAR:
	case RN5T618_CHGCTL1:
	case RN5T618_REGISET1 ... RN5T618_REGISET2:
	case RN5T618_CHGSTATE:
	case RN5T618_CHGCTRL_IRR ... RN5T618_CHGERR_MONI:
	case RN5T618_GCHGDET:
	case RN5T618_CONTROL ... RN5T618_CC_AVEREG0:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rn5t618_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_reg	= rn5t618_volatile_reg,
	.max_register	= RN5T618_MAX_REG,
	.cache_type	= REGCACHE_RBTREE,
};

static const struct regmap_irq rc5t619_irqs[] = {
	REGMAP_IRQ_REG(RN5T618_IRQ_SYS, 0, BIT(0)),
	REGMAP_IRQ_REG(RN5T618_IRQ_DCDC, 0, BIT(1)),
	REGMAP_IRQ_REG(RN5T618_IRQ_RTC, 0, BIT(2)),
	REGMAP_IRQ_REG(RN5T618_IRQ_ADC, 0, BIT(3)),
	REGMAP_IRQ_REG(RN5T618_IRQ_GPIO, 0, BIT(4)),
	REGMAP_IRQ_REG(RN5T618_IRQ_CHG, 0, BIT(6)),
};

static const struct regmap_irq_chip rc5t619_irq_chip = {
	.name = "rc5t619",
	.irqs = rc5t619_irqs,
	.num_irqs = ARRAY_SIZE(rc5t619_irqs),
	.num_regs = 1,
	.status_base = RN5T618_INTMON,
	.mask_base = RN5T618_INTEN,
	.mask_invert = true,
};

static struct i2c_client *rn5t618_pm_power_off;
static struct notifier_block rn5t618_restart_handler;

static int rn5t618_irq_init(struct rn5t618 *rn5t618)
{
	const struct regmap_irq_chip *irq_chip = NULL;
	int ret;

	if (!rn5t618->irq)
		return 0;

	switch (rn5t618->variant) {
	case RC5T619:
		irq_chip = &rc5t619_irq_chip;
		break;
	default:
		dev_err(rn5t618->dev, "Currently no IRQ support for variant %d\n",
			(int)rn5t618->variant);
		return -ENOENT;
	}

	ret = devm_regmap_add_irq_chip(rn5t618->dev, rn5t618->regmap,
				       rn5t618->irq,
				       IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				       0, irq_chip, &rn5t618->irq_data);
	if (ret)
		dev_err(rn5t618->dev, "Failed to register IRQ chip\n");

	return ret;
}

static void rn5t618_trigger_poweroff_sequence(bool repower)
{
	int ret;

	/* disable automatic repower-on */
	ret = i2c_smbus_read_byte_data(rn5t618_pm_power_off, RN5T618_REPCNT);
	if (ret < 0)
		goto err;

	ret &= ~RN5T618_REPCNT_REPWRON;
	if (repower)
		ret |= RN5T618_REPCNT_REPWRON;

	ret = i2c_smbus_write_byte_data(rn5t618_pm_power_off,
					RN5T618_REPCNT, (u8)ret);
	if (ret < 0)
		goto err;

	/* start power-off sequence */
	ret = i2c_smbus_read_byte_data(rn5t618_pm_power_off, RN5T618_SLPCNT);
	if (ret < 0)
		goto err;

	ret |= RN5T618_SLPCNT_SWPWROFF;

	ret = i2c_smbus_write_byte_data(rn5t618_pm_power_off,
					RN5T618_SLPCNT, (u8)ret);
	if (ret < 0)
		goto err;

	return;

err:
	dev_alert(&rn5t618_pm_power_off->dev, "Failed to shutdown (err = %d)\n", ret);
}

static void rn5t618_power_off(void)
{
	rn5t618_trigger_poweroff_sequence(false);
}

static int rn5t618_restart(struct notifier_block *this,
			    unsigned long mode, void *cmd)
{
	rn5t618_trigger_poweroff_sequence(true);

	/*
	 * Re-power factor detection on PMIC side is not instant. 1ms
	 * proved to be enough time until reset takes effect.
	 */
	mdelay(1);

	return NOTIFY_DONE;
}

static const struct of_device_id rn5t618_of_match[] = {
	{ .compatible = "ricoh,rn5t567", .data = (void *)RN5T567 },
	{ .compatible = "ricoh,rn5t618", .data = (void *)RN5T618 },
	{ .compatible = "ricoh,rc5t619", .data = (void *)RC5T619 },
	{ }
};
MODULE_DEVICE_TABLE(of, rn5t618_of_match);

static int rn5t618_set_repower_time(struct i2c_client *i2c)
{
	struct rn5t618 *priv = i2c_get_clientdata(i2c);
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, RN5T618_REPCNT);
	if (ret < 0)
		return ret;

	ret |= priv->repower_time;

	ret = i2c_smbus_write_byte_data(i2c, RN5T618_REPCNT, (u8)ret);
	if (ret < 0)
		return ret;

	return 0;
}

static int rn5t618_i2c_probe(struct i2c_client *i2c)
{
	const struct of_device_id *of_id;
	struct device_node *node = i2c->dev.of_node;
	const void *prop;
	int sz;
	struct rn5t618 *priv;
	u32 pmic_id;
	bool slave = false;
	int ret;

	of_id = of_match_device(rn5t618_of_match, &i2c->dev);
	if (!of_id) {
		dev_err(&i2c->dev, "Failed to find matching DT ID\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&i2c->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(i2c, priv);
	priv->variant = (long)of_id->data;
	priv->irq = i2c->irq;
	priv->dev = &i2c->dev;

	priv->regmap = devm_regmap_init_i2c(i2c, &rn5t618_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&i2c->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	slave = of_property_read_bool(node, "slave-mode");

	if (of_property_read_u32(node, "pmic-id", &pmic_id) != 0)
		pmic_id = PLATFORM_DEVID_NONE;

	if (priv->variant == RC5T619)
		ret = devm_mfd_add_devices(&i2c->dev, pmic_id,
					   rc5t619_cells,
					   ARRAY_SIZE(rc5t619_cells),
					   NULL, 0, NULL);
	else
		ret = devm_mfd_add_devices(&i2c->dev, pmic_id,
					   rn5t618_cells,
					   ARRAY_SIZE(rn5t618_cells),
					   NULL, 0, NULL);
	if (ret) {
		dev_err(&i2c->dev, "failed to add sub-devices: %d\n", ret);
		return ret;
	}

	if (of_property_read_u32(node, "repower-time", &priv->repower_time) != 0)
		priv->repower_time = REPWRTIME_10MS;

	ret = rn5t618_set_repower_time(i2c);
	if (ret) {
		dev_err(&i2c->dev, "failed to set 'repower-time': %d\n", ret);
		return ret;
	}

	prop = of_get_property(node, "suspend-sequence", &sz);
	if (prop) {
		priv->suspend_seq.seq = devm_kzalloc(&i2c->dev, sz, GFP_KERNEL);
		if (!priv->suspend_seq.seq) {
			dev_err(&i2c->dev, "cannot allocate memory for 'suspend-sequence'\n");
			return -ENOMEM;
		}

		memcpy(priv->suspend_seq.seq, prop, sz);
		priv->suspend_seq.size = sz;
	}
	else
		priv->suspend_seq.size = 0;

	prop = of_get_property(node, "resume-sequence", &sz);
	if (prop) {
		priv->resume_seq.seq = devm_kzalloc(&i2c->dev, sz, GFP_KERNEL);
		if (!priv->resume_seq.seq) {
			dev_err(&i2c->dev, "cannot allocate memory for 'resume-sequence'\n");
			return -ENOMEM;
		}

		memcpy(priv->resume_seq.seq, prop, sz);
		priv->resume_seq.size = sz;
	}
	else
		priv->resume_seq.size = 0;

	if (!slave) {
		rn5t618_pm_power_off = i2c;
		if (of_device_is_system_power_controller(i2c->dev.of_node)) {
			if (!pm_power_off)
				pm_power_off = rn5t618_power_off;
			else
				dev_warn(&i2c->dev, "Poweroff callback already assigned\n");
		}

		rn5t618_restart_handler.notifier_call = rn5t618_restart;
		rn5t618_restart_handler.priority = 192;

		ret = register_restart_handler(&rn5t618_restart_handler);
		if (ret) {
			dev_err(&i2c->dev, "cannot register restart handler, %d\n", ret);
			return ret;
		}

		dev_info(&i2c->dev, "running in single/master mode\n");
	}
	else
		dev_info(&i2c->dev, "running in slave mode\n");

	return rn5t618_irq_init(priv);
}

static int rn5t618_i2c_remove(struct i2c_client *i2c)
{
	if (i2c == rn5t618_pm_power_off) {
		rn5t618_pm_power_off = NULL;
		pm_power_off = NULL;
	}

	unregister_restart_handler(&rn5t618_restart_handler);

	return 0;
}

static int __maybe_unused rn5t618_i2c_send_sequence(struct device *dev,
		struct rn5t618_pm_sequence *seq)
{
	struct rn5t618 *priv = dev_get_drvdata(dev);
	struct rn5t618_reg_record {
		u8 reg;
		u8 value;
	} *rec;
	u8 idx, cnt;
	int ret = 0;

	cnt = seq->size / sizeof(struct rn5t618_reg_record);
	rec = seq->seq;

	for (idx=0; idx<cnt; idx++, rec++) {

		dev_dbg(dev, "reg=0x%02x, value=0x%02x\n", rec->reg, rec->value);

		ret = regmap_write(priv->regmap, rec->reg, rec->value);
		if (ret < 0) {
			dev_err(dev, "cannot set register 0x%04x to 0x%04x.\n",
					rec->reg, rec->value);
			return -EIO;
		}
	}

	return 0;
}

static int __maybe_unused rn5t618_i2c_suspend(struct device *dev)
{
	struct rn5t618 *priv = dev_get_drvdata(dev);

	if (priv->irq)
		disable_irq(priv->irq);

	if (!priv->suspend_seq.size)
	return 0;

	return rn5t618_i2c_send_sequence(dev, &priv->suspend_seq);
}

static int __maybe_unused rn5t618_i2c_resume(struct device *dev)
{
	struct rn5t618 *priv = dev_get_drvdata(dev);

	if (priv->irq)
		enable_irq(priv->irq);

	if (!priv->resume_seq.size)
	return 0;

	return rn5t618_i2c_send_sequence(dev, &priv->resume_seq);
}

static SIMPLE_DEV_PM_OPS(rn5t618_i2c_dev_pm_ops,
			rn5t618_i2c_suspend,
			rn5t618_i2c_resume);

static struct i2c_driver rn5t618_i2c_driver = {
	.driver = {
		.name = "rn5t618",
		.of_match_table = of_match_ptr(rn5t618_of_match),
		.pm = &rn5t618_i2c_dev_pm_ops,
	},
	.probe_new = rn5t618_i2c_probe,
	.remove = rn5t618_i2c_remove,
};

module_i2c_driver(rn5t618_i2c_driver);

MODULE_AUTHOR("Beniamino Galvani <b.galvani@gmail.com>");
MODULE_DESCRIPTION("Ricoh RN5T567/618 MFD driver");
MODULE_LICENSE("GPL v2");
