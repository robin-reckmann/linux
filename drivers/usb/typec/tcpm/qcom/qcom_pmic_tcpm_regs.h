/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023, Linaro Ltd. All rights reserved.
 */
#ifndef __QCOM_PMIC_TYPEC_REGS_H__
#define __QCOM_PMIC_TYPEC_REGS_H__

#include <linux/bitfield.h>
#include <linux/bug.h>
#include <linux/log2.h>

enum qcom_pmic_typec_reg {
	SNK_STATUS, // TYPE_C_STATUS_1_REG
	SRC_STATUS, // TYPE_C_STATUS_2_REG
	VBUS_STATUS, // TYPEC_SM_STATUS_REG / TYPE_C_STATUS_4_REG
	MISC_STATUS, // TYPE_C_STATUS_5_REG
	MODE_CFG, // TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG
	MODE_2_CFG, // TYPE_C_CFG_3_REG - only on pmi8998
	VCONN_CFG, // TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG
	EXIT_STATE_CFG, // TYPE_C_CFG_2_REG
	CURRSRC_CFG, // TYPE_C_CFG_2_REG only has 180ua/80ua
	/* Don't access these directly, use qptc_intr_cfg() */
	INTR_1_CFG, // TYPE_C_INTRPT_ENB_REG
	INTR_2_CFG, // TYPE_C_CFG_3_REG
};

/* SNK_STATUS */
enum qcom_pmic_typec_snk_status_fields {
	SNK_RP_STD,
#define SNK_RP_STD_VAL	BIT(2)
	SNK_RP_1P5,
#define SNK_RP_1P5_VAL	BIT(1)
	SNK_RP_3P0,
#define SNK_RP_3P0_VAL	BIT(0)
	SNK_TYPE_MASK,	/* Sink type mask */
};

/* SRC_STATUS */
enum qmic_pmic_typec_src_status_fields {
	SRC_RD_OPEN,
#define SRC_RD_OPEN_VAL	BIT(3)
	SRC_RD_RA_VCONN,
#define SRC_RD_RA_VCONN_VAL	BIT(2)
	SRC_RD_RD,
#define SRC_RD_RD_VAL		BIT(1)
	SRC_RA_RA,
#define SRC_RA_RA_VAL		BIT(0)
	SRC_TYPE_MASK,	/* Source type mask */
};

/* VBUS_STATUS */
enum qcom_pmic_typec_vbus_status_fields {
	VBUS_VSAFE5V,
	VBUS_VSAFE0V,
};

/* MISC_STATUS */
enum qcom_pmic_typec_misc_status_fields {
	WATER_DETECTION_STATUS,
	SNK_SRC_MODE,
	VBUS_DETECT,
	VBUS_ERROR_STATUS,
	DEBOUNCE_DONE,
	CC_ORIENTATION,
	CC_ATTACHED,
};

/* MODE_CFG / MODE_2_CFG */
enum qcom_pmic_typec_mode_cfg_fields {
	EN_TRY_SNK,
#define DRP_MODE_VAL 0
	EN_TRY_SRC,
	POWER_ROLE_CMD_MASK,
	EN_SRC_ONLY,
#define SRC_ONLY_MODE_VAL 1
	EN_SNK_ONLY,
#define SNK_ONLY_MODE_VAL 2
	DISABLE_CMD,
	TRY_MODE_MASK, /* PM8150b */
};

/* VCONN_CFG */
enum qcom_pmic_typec_vconn_cfg_fields {
	VCONN_EN_ORIENTATION,
	VCONN_EN_VALUE,
	VCONN_EN_SRC,
};

/* EXIT_STATE_CFG */
enum qcom_pmic_typec_exit_state_cfg_fields {
	EXIT_SNK_BASED_ON_CC,
	BYPASS_VSAFE0V_DURING_ROLE_SWAP,
	SEL_SRC_UPPER_REF,
	USE_TPD_FOR_EXITING_ATTACHSRC,
};

/* CURRSRC_CFG */
enum qcom_pmic_typec_currsrc_cfg_fields {
	CC_1P4_1P6, // Equivalent to PM8150b SEL_SRC_UPPER_REF
	SRC_RP_SEL_80UA,
#define SRC_RP_SEL_80UA_VAL 0
	SRC_RP_SEL_180UA,
#define SRC_RP_SEL_180UA_VAL BIT(0)
	SRC_RP_SEL_330UA,
#define SRC_RP_SEL_330UA_VAL BIT(1)
	SRC_RP_SEL_MASK,
};

/*
 * INTR_1_CFG / INTR_2_CFG
 * pmi8998 and pm8150b both use two registers to configure which
 * interrupts are enabled. However pm8150b shuffled the arrangement
 * The same fields enum is used for both, and they're given special
 * handling to get the correct register and bit position.
 */
enum qcom_pmic_typec_intr_cfg_fields {
	LEGACY_CABLE_INT_EN,
	NONCOMPLIANT_LEGACY_CABLE_INT_EN,
	TRYSOURCE_DETECT_INT_EN,
	TRYSINK_DETECT_INT_EN,
	CCOUT_DETACH_INT_EN,
	CCOUT_ATTACH_INT_EN,
	VBUS_DEASSERT_INT_EN,
	VBUS_ASSERT_INT_EN,
	STATE_MACHINE_CHANGE_INT_EN, // These are on INTR_2_CFG on pm8150b
	VBUS_ERROR_INT_EN,
	DEBOUNCE_DONE_INT_EN,
	CC_STATE_CHANGE_INT_EN,
};

/* Shameless "inspired" by IPA */

/*
 * PMIC registers have 16-bit addresses and 8-bit values
 * QPTC = Qcom Pmic Type-C
 */
struct qptc_reg {
	u16 offset;
	u16 n_fields;
	const u8 *fields;
	const char *name;
};

#define QPTC_REG(_NAME, _name, _offset) \
	static const struct qptc_reg qptc_reg_##_name = { \
		.offset = _offset, \
		.n_fields = ARRAY_SIZE(qptc_reg_ ## _name ## _fields), \
		.fields = qptc_reg_ ## _name ## _fields, \
		.name = #_NAME, \
	}

/*
 * struct qptc_regs - PMIC registers
 *
 * intr_1_fmask: INTR_CFG fields that are in the INTR_1 register
 * intr_2_fmask: INTR_CFG fields that are in the INTR_2 register
 * n_regs: number of registers
 * regs: array of registers
 */
struct qptc_regs {
	u32 intr_1_fmask;
	u32 intr_2_fmask;
	u32 n_regs;
	const struct qptc_reg **regs;
}; 

/* Field mask for a field or group of fields in a register */
static inline u8 qptc_reg_fmask(const struct qptc_reg *reg, u32 field_id)
{
	u8 field;
	if (!reg || WARN_ON(field_id >= reg->n_fields))
		return 0;

	field = reg->fields[field_id];

	// Field values *should* never be 0?
	WARN(!field, "Field %d is not defined in register %s\n",
		field_id, reg->name);

	return field;
}

/* Single bit field (validation) */
static inline u8 qptc_reg_bit(const struct qptc_reg *reg, u32 field_id)
{
	u32 fmask = qptc_reg_fmask(reg, field_id);

	WARN_ON(!is_power_of_2(fmask));

	return fmask;
}

/* Encode a value into a field */
static inline u8 qptc_reg_encode(const struct qptc_reg *reg, u32 field_id,
				 u8 val)
{
	u32 fmask = qptc_reg_fmask(reg, field_id);

	if (!fmask)
		return 0;

	val <<= __ffs(fmask);
	if (WARN_ON(val & ~fmask))
		return 0;

	return val;
}

/* Retrieve a field value from a register */
static inline u8 qptc_reg_decode(const struct qptc_reg *reg, u32 field_id,
				 u8 val)
{
	u32 fmask = qptc_reg_fmask(reg, field_id);

	if (!fmask)
		return 0;

	return (val & fmask) >> __ffs(fmask);
}

/* Given a mask of register fields, produce a bitmask of values */
static inline u8 qptc_reg_mask_prepare(const struct qptc_reg *reg, u32 field_mask)
{
	u32 field_bit;
	u8 mask = 0;

	while (field_mask) {
		field_bit = __ffs(field_mask);
		field_mask &= ~BIT(field_bit);

		mask |= qptc_reg_fmask(reg, field_bit);
	}

	return mask;
}

static inline u16 qptc_reg_offset(const struct qptc_reg *reg)
{
	return reg ? reg->offset : 0;
}

struct pmic_typec;
const struct qptc_reg *qptc_reg(struct pmic_typec *typec,
				enum qcom_pmic_typec_reg reg_id);

extern const struct qptc_regs qcom_pmic_typec_pm8150b_regs;
extern const struct qptc_regs qcom_pmic_typec_pmi8998_regs;

#endif /* __QCOM_PMIC_TYPEC_REGS_H__ */
