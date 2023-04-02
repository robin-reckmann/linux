// SPDX-License-Identifier: GPL-2.0-only
//
// Qualcomm PMIC VBUS output regulator driver
//
// Copyright (c) 2020, The Linux Foundation. All rights reserved.

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>
#include <soc/qcom/qcom-spmi-pmic.h>

#define OTG_STATUS_REG			0x09
#define BOOST_SOFTSTART_DONE_BIT	BIT(3)
#define OTG_STATE_MASK			GENMASK(2, 0)
#define OTG_STATE_ENABLED		0x2
#define CMD_OTG				0x40
#define OTG_EN				BIT(0)
#define OTG_CURRENT_LIMIT_CFG		0x52
#define OTG_CURRENT_LIMIT_MASK		GENMASK(2, 0)
#define OTG_CFG				0x53
#define OTG_EN_SRC_CFG			BIT(1)

#define HAS_STATUS_REG true

static const unsigned int curr_table[] = {
	500000, 1000000, 1500000, 2000000, 2500000, 3000000,
};

static const unsigned int curr_table_pmi8998[] = {
       250000, 500000, 750000, 1000000, 1250000, 1500000, 1750000, 2000000,
};

static int qcom_usb_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	unsigned int val;
	// FIXME: urgh
	u32 base = rdev->desc->enable_reg - CMD_OTG;

	/* If enable time is not defined, fall back to the default impl */
	if (!rdev->desc->enable_time)
		return regulator_is_enabled_regmap(rdev);

	ret = regmap_read(rdev->regmap, base + OTG_STATUS_REG, &val);
	if (ret < 0)
		return ret;

	return !!(val & OTG_STATE_ENABLED);
}

static const struct regulator_ops qcom_usb_vbus_reg_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = qcom_usb_vbus_regulator_is_enabled,
	.get_current_limit = regulator_get_current_limit_regmap,
	.set_current_limit = regulator_set_current_limit_regmap,
};

static struct regulator_desc qcom_usb_vbus_rdesc = {
	.name = "usb_vbus",
	.ops = &qcom_usb_vbus_reg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
       .curr_table = curr_table_pmi8998, // FIXME: make dynamic
       .n_current_limits = ARRAY_SIZE(curr_table_pmi8998),
};

irqreturn_t qcom_usb_vbus_regulator_oc_irq(int irq, void *data)
{
       struct regulator_dev *rdev = data;
       struct device *dev = rdev_get_dev(rdev);
       int ret;
       unsigned int val;

       ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &val);
       if (ret || val & OTG_EN)
               return IRQ_HANDLED;

       dev_warn(dev, "VBUS overcurrent detected\n");

       // AWFUL AWFUL AWFUL
       regmap_write(rdev->regmap, rdev->desc->enable_reg, 0);
       msleep(100);
       regmap_write(rdev->regmap, rdev->desc->enable_reg, 1);

       return IRQ_HANDLED;
};

static int qcom_usb_vbus_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	int ret, irq;
	u32 base;
	bool has_status_reg;

	ret = of_property_read_u32(dev->of_node, "reg", &base);
	if (ret < 0) {
		dev_err(dev, "no base address found\n");
		return ret;
	}

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap) {
		dev_err(dev, "Failed to get regmap\n");
		return -ENOENT;
	}

	has_status_reg = (bool)device_get_match_data(dev);

	init_data = of_get_regulator_init_data(dev, dev->of_node,
					       &qcom_usb_vbus_rdesc);
	if (!init_data)
		return -ENOMEM;

	qcom_usb_vbus_rdesc.enable_reg = base + CMD_OTG;
	qcom_usb_vbus_rdesc.enable_mask = OTG_EN;
	qcom_usb_vbus_rdesc.csel_reg = base + OTG_CURRENT_LIMIT_CFG;
	qcom_usb_vbus_rdesc.csel_mask = OTG_CURRENT_LIMIT_MASK;
	if (has_status_reg) {
		qcom_usb_vbus_rdesc.enable_time = 250000; // 2.5ms
		qcom_usb_vbus_rdesc.poll_enabled_time = 100; // 0.1ms
	}
	config.dev = dev;
	config.init_data = init_data;
	config.of_node = dev->of_node;
	config.regmap = regmap;

	rdev = devm_regulator_register(dev, &qcom_usb_vbus_rdesc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(dev, "not able to register vbus reg %d\n", ret);
		return ret;
	}

       irq = platform_get_irq_byname(pdev, "otg-overcurrent");
       ret = devm_request_threaded_irq(dev, irq, NULL, qcom_usb_vbus_regulator_oc_irq, IRQF_ONESHOT, NULL, rdev);
       if (ret)
               return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	/* Disable HW logic for VBUS enable */
	regmap_update_bits(regmap, base + OTG_CFG, OTG_EN_SRC_CFG, 0);

	return 0;
}

static const struct of_device_id qcom_usb_vbus_regulator_match[] = {
	{ .compatible = "qcom,pm8150b-vbus-reg", .data = NULL },
	{ .compatible = "qcom,pmi8998-vbus-reg", .data = (void*)HAS_STATUS_REG },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_usb_vbus_regulator_match);

static struct platform_driver qcom_usb_vbus_regulator_driver = {
	.driver		= {
		.name	= "qcom-usb-vbus-regulator",
		.of_match_table = qcom_usb_vbus_regulator_match,
	},
	.probe		= qcom_usb_vbus_regulator_probe,
};
module_platform_driver(qcom_usb_vbus_regulator_driver);

MODULE_DESCRIPTION("Qualcomm USB vbus regulator driver");
MODULE_LICENSE("GPL v2");
