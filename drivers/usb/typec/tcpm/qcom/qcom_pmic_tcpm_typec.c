// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021, Linaro Ltd. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>
#include <dt-bindings/usb/typec/tcpm/qcom,pmic-usb-typec.h>
#include <soc/qcom/qcom-spmi-pmic.h>
#include "qcom_pmic_tcpm_typec.h"
#include "qcom_pmic_tcpm_regs.h"

#define PMIC_TYPEC_MAX_IRQS     0x08

// Some dodgy workaround for a HW bug "PBS_WA" in downstream
#define PMI8998_TM_IO_DTEST4_SEL_REG                   0x16E9
#define PMI8998_TM_IO_DTEST4_SEL_CRUDE                 0xA5

struct pmic_typec_irq_params {
	int             virq;
	char                *irq_name;
	unsigned long           irq_flags;
};

struct pmic_typec_resources {
	unsigned int            nr_irqs;
	struct pmic_typec_irq_params    irq_params[PMIC_TYPEC_MAX_IRQS];
	struct qptc_regs      *regs;
	irq_handler_t           irq_handler;
	enum qcom_pmic_subtype		subtype;

};

struct pmic_typec_irq_data {
	int             virq;
	int             irq;
	struct pmic_typec       *pmic_typec;
};

struct pmic_typec {
	struct device           *dev;
	struct tcpm_port        *tcpm_port;
	struct regmap           *regmap;
	u32             base;
	unsigned int            nr_irqs;
	struct pmic_typec_irq_data  *irq_data;

	struct regulator        *vdd_vbus;

	int             cc;
	bool                debouncing_cc;
	struct delayed_work     cc_debounce_dwork;

	spinlock_t          lock;   /* Register atomicity */

	const struct qptc_regs        *regs;

	enum qcom_pmic_subtype      subtype;
};

static const char * const typec_cc_status_name[] = {
	[TYPEC_CC_OPEN]     = "Open",
	[TYPEC_CC_RA]       = "Ra",
	[TYPEC_CC_RD]       = "Rd",
	[TYPEC_CC_RP_DEF]   = "Rp-def",
	[TYPEC_CC_RP_1_5]   = "Rp-1.5",
	[TYPEC_CC_RP_3_0]   = "Rp-3.0",
};

static const char *rp_unknown = "unknown";

static const char *cc_to_name(enum typec_cc_status cc)
{
	if (cc > TYPEC_CC_RP_3_0)
		return rp_unknown;

	return typec_cc_status_name[cc];
}

static const char * const rp_sel_name[] = {
	[SRC_RP_SEL_80UA_VAL]       = "Rp-def-80uA",
	[SRC_RP_SEL_180UA_VAL]  = "Rp-1.5-180uA",
	[SRC_RP_SEL_330UA_VAL]  = "Rp-3.0-330uA",
};

static const char *rp_sel_to_name(int rp_sel)
{
	if (rp_sel > SRC_RP_SEL_330UA_VAL)
		return rp_unknown;

	return rp_sel_name[rp_sel];
}

static bool qptc_reg_valid(struct pmic_typec *pmic_typec, enum qcom_pmic_typec_reg reg_id)
{
	enum qcom_pmic_subtype subtype = pmic_typec->subtype;
	bool valid;

	if ((u32)reg_id >= pmic_typec->regs->n_regs)
		return false;
	
	switch (reg_id) {
	case VBUS_STATUS:
		valid = subtype == PM8150B_SUBTYPE;
		break;
	default:
		valid = true;
		break;
	}

	dev_info(pmic_typec->dev, "%s: valid: %d, reg_name: %s\n", __func__, valid, pmic_typec->regs->regs[reg_id] ? pmic_typec->regs->regs[reg_id]->name : "NULL");

	return valid && pmic_typec->regs->regs[reg_id];
}

const struct qptc_reg *qptc_reg(struct pmic_typec *pmic_typec,
				enum qcom_pmic_typec_reg reg_id)
{
	if (WARN_ON(!qptc_reg_valid(pmic_typec, reg_id)))
		return NULL;

	return pmic_typec->regs->regs[reg_id];
}

/*
 * The interrupt configuration spans 2 registers, which interrupt is on
 * which register varies across PMICs. This function takes a platform
 * agnostic bitmask of interrupts (enum qcom_pmic_typec_intr_cfg_fields
 * represents the bit positions)
 * and generates the platform specific masks for each register
 */
static void qptc_intr_cfg(struct pmic_typec *pmic_typec, u32 intr_mask,
			  u8 *intr_masks)
{
	const struct qptc_regs *regs = pmic_typec->regs;
	enum qcom_pmic_typec_intr_cfg_fields field;
	const struct qptc_reg *intr_regs[2];
	u32 intr_field[2] = {0, 0};
	int i;

	intr_regs[0] = qptc_reg(pmic_typec, INTR_1_CFG);
	intr_regs[1] = qptc_reg(pmic_typec, INTR_2_CFG);

	/* Figure out which fields are in which registers */
	while (intr_mask) {
		field = __ffs(intr_mask);
		intr_mask &= ~BIT(field);

		if (regs->intr_1_fmask & BIT(field))
			intr_field[0] |= BIT(field);
		else if (regs->intr_2_fmask & BIT(field))
			intr_field[1] |= BIT(field);
	}

	for(i = 0; i < 2; i++) {
		while (intr_field[i]) {
			u32 intr_bit = __ffs(intr_field[i]);
			intr_field[i] &= ~BIT(intr_bit);
			intr_masks[i] |= qptc_reg_bit(intr_regs[i],intr_bit);
		}
	}
}

#define orient_to_cc(orientation) orientation ? "cc1" : "cc2"
#define orient_to_vconn(orientation) orient_to_cc(!orientation)

static void qcom_pmic_tcpm_typec_cc_debounce(struct work_struct *work)
{
	struct pmic_typec *pmic_typec =
		container_of(work, struct pmic_typec, cc_debounce_dwork.work);
	unsigned long flags;

	spin_lock_irqsave(&pmic_typec->lock, flags);
	pmic_typec->debouncing_cc = false;
	spin_unlock_irqrestore(&pmic_typec->lock, flags);

	dev_info(pmic_typec->dev, "Debounce cc complete\n");
}

static irqreturn_t pmic_typec_isr_pmi8998(int irq, void *dev_id)
{
	struct pmic_typec_irq_data *irq_data = dev_id;
	struct pmic_typec *pmic_typec = irq_data->pmic_typec;
	bool vbus_change = false;
	bool cc_change = false;
	unsigned long flags;
	u32 usbin_sts;

	dev_info(pmic_typec->dev, "%s: irq: %d, virq: %d\n", __func__, irq, irq_data->virq);

	spin_lock_irqsave(&pmic_typec->lock, flags);

	switch (irq_data->virq) {
	case PMIC_TYPEC_CC_STATE_IRQ:
		printk(KERN_ALERT "DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);
		if (!pmic_typec->debouncing_cc)
			cc_change = true;
		vbus_change = true;
		break;
	default:
		dev_warn(pmic_typec->dev, "Unhandled interrupt %d\n",
			irq_data->virq);
		break;
	}

	spin_unlock_irqrestore(&pmic_typec->lock, flags);

	regmap_read(pmic_typec->regmap, pmic_typec->base + 0x06, &usbin_sts);
	dev_info(pmic_typec->dev, "%s: usbin_sts: 0x%x\n", __func__, usbin_sts);
	if (!usbin_sts) {
		regmap_write(pmic_typec->regmap, PMI8998_TM_IO_DTEST4_SEL_REG, PMI8998_TM_IO_DTEST4_SEL_CRUDE);
	}

	if (vbus_change)
		tcpm_vbus_change(pmic_typec->tcpm_port);

	if (cc_change)
		tcpm_cc_change(pmic_typec->tcpm_port);

	return IRQ_HANDLED;
}

static irqreturn_t pmic_typec_isr_pm8150b(int irq, void *dev_id)
{
	struct pmic_typec_irq_data *irq_data = dev_id;
	struct pmic_typec *pmic_typec = irq_data->pmic_typec;
	bool vbus_change = false;
	bool cc_change = false;
	unsigned long flags;

	spin_lock_irqsave(&pmic_typec->lock, flags);

	switch (irq_data->virq) {
	case PMIC_TYPEC_VBUS_IRQ:
		/* Incoming vbus assert/de-assert detect */
		vbus_change = true;
		break;
	case PMIC_TYPEC_CC_STATE_IRQ:
		if (!pmic_typec->debouncing_cc)
			cc_change = true;
		break;
	case PMIC_TYPEC_ATTACH_DETACH_IRQ:
		if (!pmic_typec->debouncing_cc)
			cc_change = true;
		break;
	}

	spin_unlock_irqrestore(&pmic_typec->lock, flags);

	if (vbus_change)
		tcpm_vbus_change(pmic_typec->tcpm_port);

	if (cc_change)
		tcpm_cc_change(pmic_typec->tcpm_port);

	return IRQ_HANDLED;
}

int qcom_pmic_tcpm_typec_get_vbus(struct pmic_typec *pmic_typec)
{
	struct device *dev = pmic_typec->dev;
	unsigned int misc;
	const struct qptc_reg *reg;
	u16 offset;
	bool vbus_detect;
	int ret;

	reg = qptc_reg(pmic_typec, MISC_STATUS);
	offset = qptc_reg_offset(reg);
	ret = regmap_read(pmic_typec->regmap,
			  pmic_typec->base + offset,
			  &misc);
	if (ret)
		misc = 0;

	vbus_detect = qptc_reg_decode(reg, VBUS_DETECT, misc);

	dev_info(dev, "get_vbus: 0x%08x detect %d\n", misc, vbus_detect);

	return vbus_detect;
}

// FIXME: pmi8998 doesn't have a register field analogous to this
// it will need to be handled separately
int qcom_pmic_tcpm_typec_set_vbus(struct pmic_typec *pmic_typec, bool on)
{
	const struct qptc_reg *reg;
	u32 sm_stat;
	u32 val;
	u16 offset;
	int ret;

	if (on) {
		ret = regulator_enable(pmic_typec->vdd_vbus);
		if (ret)
			return ret;
	} else if (regulator_is_enabled(pmic_typec->vdd_vbus)) {
		ret = regulator_disable(pmic_typec->vdd_vbus);
		if (ret)
			return ret;
	}

	if (pmic_typec->subtype == PM8150B_SUBTYPE) {
		reg = qptc_reg(pmic_typec, VBUS_STATUS);
		offset = qptc_reg_offset(reg);
		val = qptc_reg_bit(reg, on ? VBUS_VSAFE5V : VBUS_VSAFE0V);
		/* Poll waiting for transition to required vSafe5V or vSafe0V */
		ret = regmap_read_poll_timeout(pmic_typec->regmap,
					pmic_typec->base + offset,
					sm_stat, sm_stat & val,
					100, 250000);
		if (ret)
			dev_err(pmic_typec->dev, "vbus vsafe%dv fail\n", on ? 5 : 0);
	}

	return ret;
}
 
int qcom_pmic_tcpm_typec_set_current_limit(struct pmic_typec *pmic_typec,
				int ma)
{
	if (!ma)
		return 0;

	dev_info(pmic_typec->dev, "set_current_limit: %d\n", ma);
	return regulator_set_current_limit(pmic_typec->vdd_vbus,
		ma * 1000, ma * 1000);
}

int qcom_pmic_tcpm_typec_get_cc(struct pmic_typec *pmic_typec,
				enum typec_cc_status *cc1,
				enum typec_cc_status *cc2)
{
	struct device *dev = pmic_typec->dev;
	const struct qptc_reg *reg;
	unsigned int misc, val, val_decode;
	u16 offset;
	bool attached;
	bool orientation;
	int ret = 0;

	reg = qptc_reg(pmic_typec, MISC_STATUS);
	offset = qptc_reg_offset(reg);

	ret = regmap_read(pmic_typec->regmap,
			  pmic_typec->base + offset, &misc);
	if (ret)
		goto done;

	attached = qptc_reg_decode(reg, CC_ATTACHED, misc);
	orientation = qptc_reg_decode(reg, CC_ORIENTATION, misc);

	if (pmic_typec->debouncing_cc) {
		ret = -EBUSY;
		goto done;
	}

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	if (!(attached))
		goto done;

	if (qptc_reg_decode(reg, SNK_SRC_MODE, misc)) {
		reg = qptc_reg(pmic_typec, SRC_STATUS);
		offset = qptc_reg_offset(reg);
		ret = regmap_read(pmic_typec->regmap,
				  pmic_typec->base + offset,
				  &val);
		if (ret)
			goto done;
		val_decode = qptc_reg_decode(reg, SRC_TYPE_MASK, val);
		switch (val_decode) {
		case SRC_RD_OPEN_VAL:
			val = TYPEC_CC_RD;
			break;
		case SRC_RD_RA_VCONN_VAL:
			val = TYPEC_CC_RD;
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			break;
		default:
			dev_warn(dev, "unexpected src status %.2x\n", val);
			val = TYPEC_CC_RD;
			break;
		}
	} else {
		reg = qptc_reg(pmic_typec, SNK_STATUS);
		offset = qptc_reg_offset(reg);
		ret = regmap_read(pmic_typec->regmap,
				  pmic_typec->base + offset,
				  &val);
		if (ret)
			goto done;
		val_decode = qptc_reg_decode(reg, SNK_TYPE_MASK, val);
		switch (val_decode) {
		case SNK_RP_STD_VAL:
			val = TYPEC_CC_RP_DEF;
			break;
		case SNK_RP_1P5_VAL:
			val = TYPEC_CC_RP_1_5;
			break;
		case SNK_RP_3P0_VAL:
			val = TYPEC_CC_RP_3_0;
			break;
		default:
			dev_warn(dev, "unexpected snk status %.2x\n", val);
			val = TYPEC_CC_RP_DEF;
			break;
		}
		val = TYPEC_CC_RP_DEF;
	}

	if (orientation)
		*cc2 = val;
	else
		*cc1 = val;

done:
	dev_info(dev, "get_cc: misc 0x%08x cc1 0x%08x %s cc2 0x%08x %s attached %d cc=%s\n",
		misc, *cc1, cc_to_name(*cc1), *cc2, cc_to_name(*cc2), attached,
		orient_to_cc(orientation));

	return ret;
}

static void qcom_pmic_set_cc_debounce(struct pmic_typec *pmic_typec)
{
	pmic_typec->debouncing_cc = true;
	schedule_delayed_work(&pmic_typec->cc_debounce_dwork,
				  msecs_to_jiffies(2));
}

int qcom_pmic_tcpm_typec_set_cc(struct pmic_typec *pmic_typec,
				enum typec_cc_status cc)
{
	struct device *dev = pmic_typec->dev;
	unsigned int mode, currsrc = 0;
	const struct qptc_reg *reg;
	unsigned int misc;
	unsigned long flags;
	bool orientation = false, attached = false;
	u16 offset;
	int ret;

	reg = qptc_reg(pmic_typec, MISC_STATUS);
	offset = qptc_reg_offset(reg);

	spin_lock_irqsave(&pmic_typec->lock, flags);

	ret = regmap_read(pmic_typec->regmap,
			  pmic_typec->base + offset,
			  &misc);
	if (ret)
		goto done;
	
	orientation = qptc_reg_decode(reg, CC_ORIENTATION, misc);
	attached = qptc_reg_decode(reg, CC_ATTACHED, misc);

	mode = EN_SRC_ONLY;

	switch (cc) {
	case TYPEC_CC_OPEN:
		currsrc = SRC_RP_SEL_80UA_VAL;
		break;
	case TYPEC_CC_RP_DEF:
		currsrc = SRC_RP_SEL_80UA_VAL;
		break;
	case TYPEC_CC_RP_1_5:
		currsrc = SRC_RP_SEL_180UA_VAL;
		break;
	case TYPEC_CC_RP_3_0:
		// FIXME: is this right?? PMi8998 has no 330ua register
		if (pmic_typec->subtype == PMI8998_SUBTYPE)
			currsrc = SRC_RP_SEL_180UA_VAL;
		else
			currsrc = SRC_RP_SEL_330UA_VAL;
		break;
	case TYPEC_CC_RD:
		currsrc = SRC_RP_SEL_80UA_VAL;
		mode = EN_SNK_ONLY;
		break;
	default:
		dev_warn(dev, "unexpected set_cc %d\n", cc);
		ret = -EINVAL;
		goto done;
	}

	if (mode == EN_SRC_ONLY) {
		reg = qptc_reg(pmic_typec, CURRSRC_CFG);
		offset = qptc_reg_offset(reg);
		currsrc = qptc_reg_encode(reg, SRC_RP_SEL_MASK, currsrc);
		ret = regmap_write(pmic_typec->regmap,
				   pmic_typec->base + offset,
				   currsrc);
		if (ret)
			goto done;
	}

	pmic_typec->cc = cc;
	qcom_pmic_set_cc_debounce(pmic_typec);
	ret = 0;

done:
	spin_unlock_irqrestore(&pmic_typec->lock, flags);

	dev_info(dev, "set_cc: currsrc=%x %s mode %s debounce %d attached %d cc=%s\n",
		currsrc, rp_sel_to_name(currsrc),
		mode == EN_SRC_ONLY ? "EN_SRC_ONLY" : "EN_SNK_ONLY",
		pmic_typec->debouncing_cc, attached,
		orient_to_cc(orientation));

	return ret;
}

int qcom_pmic_tcpm_typec_set_vconn(struct pmic_typec *pmic_typec, bool on)
{
	struct device *dev = pmic_typec->dev;
	unsigned int vconn_orient, misc, mask, value;
	const struct qptc_reg *reg;
	u16 offset;
	unsigned long flags;
	int ret;

	reg = qptc_reg(pmic_typec, MISC_STATUS);
	offset = qptc_reg_offset(reg);
	spin_lock_irqsave(&pmic_typec->lock, flags);

	ret = regmap_read(pmic_typec->regmap,
			  pmic_typec->base + offset, &misc);
	if (ret)
		goto done;

	/* Set VCONN on the inversion of the active CC channel */
	vconn_orient = !qptc_reg_decode(reg, CC_ORIENTATION, misc);

	reg = qptc_reg(pmic_typec, VCONN_CFG);
	offset = qptc_reg_offset(reg);
	if (on) {
		mask = qptc_reg_mask_prepare(reg,
						 BIT(VCONN_EN_ORIENTATION) |
						 BIT(VCONN_EN_VALUE));
		value = qptc_reg_mask_prepare(reg, BIT(VCONN_EN_VALUE) | BIT(VCONN_EN_SRC));
		value |= qptc_reg_encode(reg, VCONN_EN_ORIENTATION, vconn_orient);
	} else {
		mask = qptc_reg_mask_prepare(reg, BIT(VCONN_EN_VALUE));
		value = 0;
	}

	ret = regmap_update_bits(pmic_typec->regmap,
				 pmic_typec->base + offset,
				 mask, value);
done:
	spin_unlock_irqrestore(&pmic_typec->lock, flags);

	dev_info(dev, "set_vconn: orientation %d control 0x%08x state %s cc %s vconn %s\n",
		vconn_orient, value, on ? "on" : "off", orient_to_vconn(!vconn_orient), orient_to_cc(!vconn_orient));

	return ret;
}

int qcom_pmic_tcpm_typec_start_toggling(struct pmic_typec *pmic_typec,
					enum typec_port_type port_type,
					enum typec_cc_status cc)
{
	struct device *dev = pmic_typec->dev;
	unsigned int misc;
	const struct qptc_reg *reg;
	u16 offset;
	u8 mode = 0;
	u8 val = 0, mask = 0;
	bool attached;
	unsigned long flags;
	int ret;

	printk(KERN_ALERT "DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	switch (port_type) {
	case TYPEC_PORT_SRC:
		mode = SRC_ONLY_MODE_VAL;
		break;
	case TYPEC_PORT_SNK:
		mode = SNK_ONLY_MODE_VAL;
		break;
	case TYPEC_PORT_DRP:
		mode = DRP_MODE_VAL;
		break;
	}

	reg = qptc_reg(pmic_typec, MISC_STATUS);
	offset = qptc_reg_offset(reg);

	spin_lock_irqsave(&pmic_typec->lock, flags);

	ret = regmap_read(pmic_typec->regmap,
			  pmic_typec->base + offset, &misc);
	if (ret)
		goto done;
	
	attached = qptc_reg_decode(reg, CC_ATTACHED, misc);

	dev_info(dev, "start_toggling: misc 0x%02x attached %d port_type %d current cc %d new %d\n",
		(u8)misc, attached, port_type, pmic_typec->cc, cc);

	qcom_pmic_set_cc_debounce(pmic_typec);

	reg = qptc_reg(pmic_typec, MODE_CFG);
	offset = qptc_reg_offset(reg);

	if (pmic_typec->subtype != PMI8998_SUBTYPE) {
		/* force it to toggle at least once */
		val = qptc_reg_encode(reg, DISABLE_CMD, 1);
		ret = regmap_update_bits(pmic_typec->regmap,
				   pmic_typec->base + offset,
				   val, val);
		if (ret)
			goto done;
	}
	mask = qptc_reg_mask_prepare(reg, BIT(POWER_ROLE_CMD_MASK));
	val = qptc_reg_encode(reg, POWER_ROLE_CMD_MASK, mode);
	dev_info(dev, "start_toggling: mask: 0x%02x, val: 0x%02x\n", mask, val);
	ret = regmap_update_bits(pmic_typec->regmap,
			   pmic_typec->base + offset,
			   mask, val);
done:
	spin_unlock_irqrestore(&pmic_typec->lock, flags);

	return ret;
}

static int qptc_configure_cc_threshold(struct pmic_typec *pmic_typec)
{
	const struct qptc_reg *reg;
	u16 offset;
	u32 mask;
	int ret;

	if (pmic_typec->subtype == PMI8998_SUBTYPE) {
			/* Set CC threshold to 1.6 Volts | tPDdebounce = 10-20ms */
			reg = qptc_reg(pmic_typec, CURRSRC_CFG);
			offset = qptc_reg_offset(reg);
			mask = qptc_reg_mask_prepare(reg, BIT(CC_1P4_1P6));
			ret = regmap_update_bits(pmic_typec->regmap,
										pmic_typec->base + offset,
										mask, mask);
	} else {
			/* Set CC threshold to 1.6 Volts | tPDdebounce = 10-20ms */
			reg = qptc_reg(pmic_typec, EXIT_STATE_CFG);
			offset = qptc_reg_offset(reg);
			mask = qptc_reg_mask_prepare(reg, BIT(SEL_SRC_UPPER_REF) | BIT(USE_TPD_FOR_EXITING_ATTACHSRC));
			ret = regmap_update_bits(pmic_typec->regmap,
										pmic_typec->base + offset,
										mask, mask);
	}

	return ret;
}

#define TYPEC_INTR_EN_CFG_MASK            \
	(BIT(LEGACY_CABLE_INT_EN)       | \
	 BIT(NONCOMPLIANT_LEGACY_CABLE_INT_EN)  | \
	 BIT(TRYSOURCE_DETECT_INT_EN)       | \
	 BIT(TRYSINK_DETECT_INT_EN)     | \
	 BIT(CCOUT_DETACH_INT_EN)       | \
	 BIT(CCOUT_ATTACH_INT_EN)       | \
	 BIT(VBUS_DEASSERT_INT_EN)      | \
	 BIT(VBUS_ASSERT_INT_EN)        | \
	 BIT(STATE_MACHINE_CHANGE_INT_EN)   | \
	 BIT(CC_STATE_CHANGE_INT_EN)        | \
	 BIT(VBUS_ERROR_INT_EN)         | \
	 BIT(DEBOUNCE_DONE_INT_EN))

int qcom_pmic_tcpm_typec_init(struct pmic_typec *pmic_typec,
				  struct tcpm_port *tcpm_port)
{
	const struct qptc_reg *reg;
	u16 offset;
	u32 mask;
	int ret;
	int i;
	u8 intr_masks[2];

	qptc_intr_cfg(pmic_typec, TYPEC_INTR_EN_CFG_MASK, intr_masks);

	reg = qptc_reg(pmic_typec, INTR_1_CFG);
	offset = qptc_reg_offset(reg);

	/* Configure interrupt sources */
	ret = regmap_write(pmic_typec->regmap,
			   pmic_typec->base + offset,
			   intr_masks[0]);
	if (ret)
		goto done;

	reg = qptc_reg(pmic_typec, INTR_2_CFG);
	offset = qptc_reg_offset(reg);
	ret = regmap_write(pmic_typec->regmap,
			   pmic_typec->base + offset,
			   intr_masks[1]);
	if (ret)
		goto done;

	/* start in TRY_SNK mode */
	// FIXME: pmi8998 downstream explicitly says "disable try.sink", investigate
	if (pmic_typec->subtype == PMI8998_SUBTYPE) {
		reg = qptc_reg(pmic_typec, MODE_2_CFG);
		offset = qptc_reg_offset(reg);
		ret = regmap_write(pmic_typec->regmap,
				pmic_typec->base + offset, qptc_reg_encode(reg, EN_TRY_SNK, 0));
		if (ret)
			goto done;
	} else {
		reg = qptc_reg(pmic_typec, MODE_CFG);
		offset = qptc_reg_offset(reg);
		ret = regmap_write(pmic_typec->regmap,
				pmic_typec->base + offset, qptc_reg_encode(reg, EN_TRY_SNK, 1));
		if (ret)
			goto done;
	}

	/* Configure VCONN for software control */
	reg = qptc_reg(pmic_typec, VCONN_CFG);
	offset = qptc_reg_offset(reg);
	mask = qptc_reg_mask_prepare(reg, BIT(VCONN_EN_SRC) | BIT(VCONN_EN_VALUE));
	ret = regmap_update_bits(pmic_typec->regmap,
				 pmic_typec->base + offset,
				 mask, qptc_reg_encode(reg, VCONN_EN_SRC, 1));
	if (ret)
		goto done;

	ret = qptc_configure_cc_threshold(pmic_typec);

	pmic_typec->tcpm_port = tcpm_port;

	for (i = 0; i < pmic_typec->nr_irqs; i++)
		enable_irq(pmic_typec->irq_data[i].irq);

done:
	return ret;
}

void qcom_pmic_tcpm_typec_put(struct pmic_typec *pmic_typec)
{
	put_device(pmic_typec->dev);
}

static inline void dump_reg_fields(struct pmic_typec *pmic_typec, const struct qptc_reg *reg)
{
	int i;

	for (i = 0; i < reg->n_fields; i++) {
			u8 field = reg->fields[i];
			if (!field)
				continue;

			dev_info(pmic_typec->dev, "\t0x%02d: 0x%02x\n", i, field);
	}
}

static void dump_reg_info(struct pmic_typec *pmic_typec) {
	const struct qptc_regs *regs = pmic_typec->regs;
	int i;

	for (i = 0; i < regs->n_regs; i++) {
			const struct qptc_reg *reg = regs->regs[i];
			if (!reg)
					continue;

			dev_info(pmic_typec->dev, "%2d: reg %16s: offset 0x%04x\n",
						i, reg->name, reg->offset);
			
			dump_reg_fields(pmic_typec, reg);
	}
}

static int qcom_pmic_tcpm_typec_probe(struct platform_device *pdev)
{
	struct pmic_typec *pmic_typec;
	struct device *dev = &pdev->dev;
	const struct pmic_typec_resources *res;
	struct pmic_typec_irq_data *irq_data;
	int i, ret, irq;
	u32 reg;

	ret = device_property_read_u32(dev, "reg", &reg);
	if (ret < 0) {
		dev_err(dev, "missing base address\n");
		return ret;
	}

	res = of_device_get_match_data(dev);
	if (!res)
		return -ENODEV;

	if (!res->nr_irqs || res->nr_irqs > PMIC_TYPEC_MAX_IRQS)
		return -EINVAL;

	pmic_typec = devm_kzalloc(dev, sizeof(*pmic_typec), GFP_KERNEL);
	if (!pmic_typec)
		return -ENOMEM;

	irq_data = devm_kzalloc(dev, sizeof(*irq_data) * res->nr_irqs,
				GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;

	pmic_typec->vdd_vbus = devm_regulator_get(dev, "vdd-vbus");
	if (IS_ERR(pmic_typec->vdd_vbus))
		return PTR_ERR(pmic_typec->vdd_vbus);

	pmic_typec->dev = dev;
	pmic_typec->base = reg;
	pmic_typec->nr_irqs = res->nr_irqs;
	pmic_typec->irq_data = irq_data;
	pmic_typec->regs = res->regs;
	pmic_typec->subtype = res->subtype;

	dump_reg_info(pmic_typec);

	spin_lock_init(&pmic_typec->lock);
	INIT_DELAYED_WORK(&pmic_typec->cc_debounce_dwork,
			  qcom_pmic_tcpm_typec_cc_debounce);

	pmic_typec->regmap = dev_get_regmap(dev->parent, NULL);
	if (!pmic_typec->regmap) {
		dev_err(dev, "Failed to get regmap\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	platform_set_drvdata(pdev, pmic_typec);

	for (i = 0; i < res->nr_irqs; i++, irq_data++) {
		irq = platform_get_irq_byname(pdev,
						  res->irq_params[i].irq_name);
		if (irq < 0)
			return irq;

		irq_data->pmic_typec = pmic_typec;
		irq_data->irq = irq;
		irq_data->virq = res->irq_params[i].virq;
		ret = devm_request_threaded_irq(dev, irq, NULL, res->irq_handler,
						IRQF_ONESHOT | IRQF_NO_AUTOEN |
						res->irq_params[i].irq_flags,
						res->irq_params[i].irq_name,
						irq_data);
		if (ret)
			return ret;
	}

	return 0;
}

static struct pmic_typec_resources pm8150b_typec_res = {
	.irq_params = {
		{
			.irq_name = "vpd-detect",
			.virq = PMIC_TYPEC_VPD_IRQ,
		},

		{
			.irq_name = "cc-state-change",
			.virq = PMIC_TYPEC_CC_STATE_IRQ, // used
		},
		{
			.irq_name = "vconn-oc",
			.virq = PMIC_TYPEC_VCONN_OC_IRQ,
		},

		{
			.irq_name = "vbus-change",
			.virq = PMIC_TYPEC_VBUS_IRQ, // used
		},

		{
			.irq_name = "attach-detach",
			.virq = PMIC_TYPEC_ATTACH_DETACH_IRQ, // used
		},
		{
			.irq_name = "legacy-cable-detect",
			.virq = PMIC_TYPEC_LEGACY_CABLE_IRQ,
		},

		{
			.irq_name = "try-snk-src-detect",
			.virq = PMIC_TYPEC_TRY_SNK_SRC_IRQ,
		},
	},
	.nr_irqs = 7,
	.regs = &qcom_pmic_typec_pm8150b_regs,
	.irq_handler = pmic_typec_isr_pm8150b,
	.subtype = PM8150B_SUBTYPE,
};

static struct pmic_typec_resources pmi8998_typec_res = {
	.irq_params = {
		{
			/*
			 * on pmi8998 this IRQ is used for most things
			 * See TYPE_C_INTRPT_ENB_REG:
			 * - TYPEC_CCOUT_DETACH_INT_EN_BIT
			 * - TYPEC_CCOUT_ATTACH_INT_EN_BIT
			 * - TYPEC_VBUS_ERROR_INT_EN_BIT
			 * - TYPEC_UFP_AUDIOADAPT_INT_EN_BIT
			 * - TYPEC_DEBOUNCE_DONE_INT_EN_BIT
			 * - TYPEC_CCSTATE_CHANGE_INT_EN_BIT
			 * - TYPEC_VBUS_DEASSERT_INT_EN_BIT
			 * - TYPEC_VBUS_ASSERT_INT_EN_BIT
			 *
			 * This irq is probably also used for
			 * PMIC_TYPEC_VBUS_IRQ
			 */
			.irq_name = "type-c-change",
			.virq = PMIC_TYPEC_CC_STATE_IRQ,
			.irq_flags = 0,
		},
		{
			/* usb-plugin IRQ, shared with charger */
			.irq_name = "attach-detach",
			.virq = PMIC_TYPEC_VBUS_IRQ,
			.irq_flags = 0,
		},
	},
	.nr_irqs = 2,
	.regs = &qcom_pmic_typec_pmi8998_regs,
	.irq_handler = pmic_typec_isr_pmi8998,
	.subtype = PMI8998_SUBTYPE,
};

static const struct of_device_id qcom_pmic_tcpm_typec_table[] = {
	{ .compatible = "qcom,pm8150b-typec", .data = &pm8150b_typec_res },
	{ .compatible = "qcom,pmi8998-typec", .data = &pmi8998_typec_res },
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_pmic_tcpm_typec_table);

struct platform_driver qcom_pmic_tcpm_typec_platform_driver = {
	.driver = {
		.name = "qcom,pmic-typec",
		.of_match_table = qcom_pmic_tcpm_typec_table,
	},
	.probe = qcom_pmic_tcpm_typec_probe,
};
