/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2022 Linaro. All rights reserved.
 * Author: Caleb Connolly <caleb.connolly@linaro.org>
 */

#ifndef __QCOM_SPMI_PMIC_H__
#define __QCOM_SPMI_PMIC_H__

#include <linux/device.h>

enum qcom_pmic_subtype {
	COMMON_SUBTYPE = 0x00,
	PM8941_SUBTYPE = 0x01,
	PM8841_SUBTYPE = 0x02,
	PM8019_SUBTYPE = 0x03,
	PM8226_SUBTYPE = 0x04,
	PM8110_SUBTYPE = 0x05,
	PMA8084_SUBTYPE = 0x06,
	PMI8962_SUBTYPE = 0x07,
	PMD9635_SUBTYPE = 0x08,
	PM8994_SUBTYPE = 0x09,
	PMI8994_SUBTYPE = 0x0a,
	PM8916_SUBTYPE = 0x0b,
	PM8004_SUBTYPE = 0x0c,
	PM8909_SUBTYPE = 0x0d,
	PM8028_SUBTYPE = 0x0e,
	PM8901_SUBTYPE = 0x0f,
	PM8950_SUBTYPE = 0x10,
	PMI8950_SUBTYPE = 0x11,
	PMK8001_SUBTYPE = 0x12,
	PMI8996_SUBTYPE = 0x13,
	PM8998_SUBTYPE = 0x14,
	PMI8998_SUBTYPE = 0x15,
	PM8005_SUBTYPE = 0x18,
	PM660L_SUBTYPE = 0x1a,
	PM660_SUBTYPE = 0x1b,
	PM8150_SUBTYPE = 0x1e,
	PM8150L_SUBTYPE = 0x1f,
	PM8150B_SUBTYPE = 0x20,
	PMK8002_SUBTYPE = 0x21,
	PM8009_SUBTYPE = 0x24,
	PMI632_SUBTYPE = 0x25,
	PM8150C_SUBTYPE = 0x26,
	PM6150_SUBTYPE = 0x28,
	SMB2351_SUBTYPE = 0x29,
	PM8008_SUBTYPE = 0x2c,
	PM6125_SUBTYPE = 0x2d,
	PM7250B_SUBTYPE = 0x2e,
	PMK8350_SUBTYPE = 0x2f,
	PMR735B_SUBTYPE = 0x34,
	PM6350_SUBTYPE = 0x36,
	PM2250_SUBTYPE = 0x37,
};

#define PMI8998_FAB_ID_SMIC	0x11
#define PMI8998_FAB_ID_GF	0x30

#define PM660_FAB_ID_GF		0x0
#define PM660_FAB_ID_TSMC	0x2
#define PM660_FAB_ID_MX		0x3

struct qcom_spmi_pmic {
	unsigned int type;
	unsigned int subtype;
	unsigned int major;
	unsigned int minor;
	unsigned int rev2;
	unsigned int fab_id;
	const char *name;
};

const struct qcom_spmi_pmic *qcom_pmic_get(struct device *dev);

#endif /* __QCOM_SPMI_PMIC_H__ */
