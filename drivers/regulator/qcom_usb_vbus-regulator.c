// SPDX-License-Identifier: GPL-2.0-only
//
// Qualcomm PMIC VBUS output regulator driver
//
// Copyright (c) 2020, The Linux Foundation. All rights reserved.

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

#define CMD_OTG				0x40
#define OTG_EN				BIT(0)
#define OTG_CURRENT_LIMIT_CFG		0x52
#define OTG_CURRENT_LIMIT_MASK		GENMASK(2, 0)
#define OTG_CFG				0x53
#define OTG_EN_SRC_CFG			BIT(1)

struct msm_vbus_desc {
	const unsigned int *curr_table;
	unsigned int n_current_limits;
};

static const unsigned int curr_table_pm8150b[] = {
	500000, 1000000, 1500000, 2000000, 2500000, 3000000,
};

static const unsigned int curr_table_pmi8998[] = {
	250000, 500000, 750000, 1000000,
	1250000, 1500000, 1750000, 2000000,
};

static const struct msm_vbus_desc msm_vbus_desc_pm8150b = {
	.curr_table = curr_table_pm8150b,
	.n_current_limits = ARRAY_SIZE(curr_table_pm8150b),
};

static const struct msm_vbus_desc msm_vbus_desc_pmi8998 = {
	.curr_table = curr_table_pmi8998,
	.n_current_limits = ARRAY_SIZE(curr_table_pmi8998),
};

static const struct regulator_ops qcom_usb_vbus_reg_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_current_limit = regulator_get_current_limit_regmap,
	.set_current_limit = regulator_set_current_limit_regmap,
};

static struct regulator_desc qcom_usb_vbus_rdesc = {
	.name = "usb_vbus",
	.ops = &qcom_usb_vbus_reg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
};

static int qcom_usb_vbus_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	const struct msm_vbus_desc *quirks;
	int ret;
	u32 base;

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

	init_data = of_get_regulator_init_data(dev, dev->of_node,
					       &qcom_usb_vbus_rdesc);
	if (!init_data)
		return -ENOMEM;

	quirks = of_device_get_match_data(dev);
	if (!quirks)
		return -ENODEV;

	qcom_usb_vbus_rdesc.curr_table = quirks->curr_table;
	qcom_usb_vbus_rdesc.n_current_limits = quirks->n_current_limits;
	qcom_usb_vbus_rdesc.enable_reg = base + CMD_OTG;
	qcom_usb_vbus_rdesc.enable_mask = OTG_EN;
	qcom_usb_vbus_rdesc.csel_reg = base + OTG_CURRENT_LIMIT_CFG;
	qcom_usb_vbus_rdesc.csel_mask = OTG_CURRENT_LIMIT_MASK;
	config.dev = dev;
	config.init_data = init_data;
	config.of_node = dev->of_node;
	config.regmap = regmap;

	rdev = devm_regulator_register(dev, &qcom_usb_vbus_rdesc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(dev, "Failed to register vbus reg %d\n", ret);
		return ret;
	}

	/* Disable HW logic for VBUS enable */
	regmap_update_bits(regmap, base + OTG_CFG, OTG_EN_SRC_CFG, 0);

	dev_dbg(dev, "Registered QCOM VBUS regulator\n");

	return 0;
}

static const struct of_device_id qcom_usb_vbus_regulator_match[] = {
	{ .compatible = "qcom,pm8150b-vbus-reg", .data = &msm_vbus_desc_pm8150b },
	{ .compatible = "qcom,pmi8998-vbus-reg", .data = &msm_vbus_desc_pmi8998 },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_usb_vbus_regulator_match);

static struct platform_driver qcom_usb_vbus_regulator_driver = {
	.driver		= {
		.name	= "qcom-usb-vbus-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = qcom_usb_vbus_regulator_match,
	},
	.probe		= qcom_usb_vbus_regulator_probe,
};
module_platform_driver(qcom_usb_vbus_regulator_driver);

MODULE_DESCRIPTION("Qualcomm USB vbus regulator driver");
MODULE_LICENSE("GPL v2");
