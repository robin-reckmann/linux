// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023, Linaro Ltd. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

#include "qcom_pmic_typec.h"
#include "qcom_pmic_typec_port.h"

#define PMI8998_USBIN_INPUT_STATUS			0x06
#define PMI8998_USBIN_INPUT_STATUS_7		BIT(7)
#define PMI8998_USBIN_INPUT_STATUS_6		BIT(6)
#define PMI8998_USBIN_12V				BIT(5)
#define PMI8998_USBIN_9V_TO_12V			BIT(4)
#define PMI8998_USBIN_9V				BIT(3)
#define PMI8998_USBIN_5V_TO_12V			BIT(2)
#define PMI8998_USBIN_5V_TO_9V			BIT(1)
#define PMI8998_USBIN_5V				BIT(0)
#define PMI8998_QC_2P0_STATUS_MASK			GENMASK(2, 0)

#define PMI8998_APSD_STATUS				0x07
#define PMI8998_APSD_STATUS_7			BIT(7)
#define PMI8998_HVDCP_CHECK_TIMEOUT			BIT(6)
#define PMI8998_SLOW_PLUGIN_TIMEOUT			BIT(5)
#define PMI8998_ENUMERATION_DONE			BIT(4)
#define PMI8998_VADP_CHANGE_DONE_AFTER_AUTH		BIT(3)
#define PMI8998_QC_AUTH_DONE_STATUS			BIT(2)
#define PMI8998_QC_CHARGER				BIT(1)
#define PMI8998_APSD_DTC_STATUS_DONE		BIT(0)

#define PMI8998_APSD_RESULT_STATUS			0x08
#define PMI8998_ICL_OVERRIDE_LATCH			BIT(7)
#define PMI8998_APSD_RESULT_STATUS_MASK			GENMASK(6, 0)
#define PMI8998_QC_3P0				BIT(6)
#define PMI8998_QC_2P0				BIT(5)
#define PMI8998_FLOAT_CHARGER			BIT(4)
#define PMI8998_DCP_CHARGER				BIT(3)
#define PMI8998_CDP_CHARGER				BIT(2)
#define PMI8998_OCP_CHARGER				BIT(1)
#define PMI8998_SDP_CHARGER				BIT(0)

#define PMI8998_QC_CHANGE_STATUS			0x09
#define PMI8998_QC_CHANGE_STATUS_7			BIT(7)
#define PMI8998_QC_CHANGE_STATUS_6			BIT(6)
#define PMI8998_QC_9V_TO_12V_REASON			BIT(5)
#define PMI8998_QC_5V_TO_9V_REASON			BIT(4)
#define PMI8998_QC_CONTINUOUS			BIT(3)
#define PMI8998_QC_12V				BIT(2)
#define PMI8998_QC_9V				BIT(1)
#define PMI8998_QC_5V				BIT(0)

#define PMI8998_QC_PULSE_COUNT_STATUS		0x0A
#define PMI8998_QC_PULSE_COUNT_STATUS_7		BIT(7)
#define PMI8998_QC_PULSE_COUNT_STATUS_6		BIT(6)
#define PMI8998_QC_PULSE_COUNT_MASK			GENMASK(5, 0)

#define PMI8998_TYPE_C_STATUS_1			0x0B
#define PMI8998_UFP_TYPEC_MASK				GENMASK(7, 5)
#define PMI8998_UFP_TYPEC_RDSTD			BIT(7)
#define PMI8998_UFP_TYPEC_RD1P5			BIT(6)
#define PMI8998_UFP_TYPEC_RD3P0			BIT(5)
#define PMI8998_UFP_TYPEC_FMB_255K			BIT(4)
#define PMI8998_UFP_TYPEC_FMB_301K			BIT(3)
#define PMI8998_UFP_TYPEC_FMB_523K			BIT(2)
#define PMI8998_UFP_TYPEC_FMB_619K			BIT(1)
#define PMI8998_UFP_TYPEC_OPEN_OPEN			BIT(0)

#define PMI8998_TYPE_C_STATUS_2			0x0C
#define PMI8998_DFP_RA_OPEN				BIT(7)
#define PMI8998_TIMER_STAGE				BIT(6)
#define PMI8998_EXIT_UFP_MODE			BIT(5)
#define PMI8998_EXIT_DFP_MODE			BIT(4)
#define PMI8998_DFP_TYPEC_MASK				GENMASK(3, 0)
#define PMI8998_DFP_RD_OPEN				BIT(3)
#define PMI8998_DFP_RD_RA_VCONN			BIT(2)
#define PMI8998_DFP_RD_RD				BIT(1)
#define PMI8998_DFP_RA_RA				BIT(0)

#define PMI8998_TYPE_C_STATUS_3			0x0D
#define PMI8998_ENABLE_BANDGAP			BIT(7)
#define PMI8998_U_USB_GND_NOVBUS			BIT(6)
#define PMI8998_U_USB_FLOAT_NOVBUS			BIT(5)
#define PMI8998_U_USB_GND				BIT(4)
#define PMI8998_U_USB_FMB1				BIT(3)
#define PMI8998_U_USB_FLOAT1			BIT(2)
#define PMI8998_U_USB_FMB2				BIT(1)
#define PMI8998_U_USB_FLOAT2			BIT(0)

#define PMI8998_TYPE_C_STATUS_4			0x0E
#define PMI8998_UFP_DFP_MODE_STATUS			BIT(7)
#define PMI8998_TYPEC_VBUS_STATUS			BIT(6)
#define PMI8998_TYPEC_VBUS_ERROR_STATUS		BIT(5)
#define PMI8998_TYPEC_DEBOUNCE_DONE_STATUS		BIT(4)
#define PMI8998_TYPEC_UFP_AUDIO_ADAPT_STATUS	BIT(3)
#define PMI8998_TYPEC_VCONN_OVERCURR_STATUS		BIT(2)
#define PMI8998_CC_ORIENTATION			BIT(1)
#define PMI8998_CC_ATTACHED				BIT(0)

#define PMI8998_TYPE_C_STATUS_5			0x0F
#define PMI8998_TRY_SOURCE_FAILED			BIT(6)
#define PMI8998_TRY_SINK_FAILED			BIT(5)
#define PMI8998_TIMER_STAGE_2			BIT(4)
#define PMI8998_TYPEC_LEGACY_CABLE_STATUS		BIT(3)
#define PMI8998_TYPEC_NONCOMP_LEGACY_CABLE_STATUS	BIT(2)
#define PMI8998_TYPEC_TRYSOURCE_DETECT_STATUS	BIT(1)
#define PMI8998_TYPEC_TRYSINK_DETECT_STATUS		BIT(0)

/* USBIN Interrupt Bits */
#define PMI8998_TYPE_C_CHANGE_RT_STS		BIT(7)
#define PMI8998_USBIN_ICL_CHANGE_RT_STS		BIT(6)
#define PMI8998_USBIN_SOURCE_CHANGE_RT_STS		BIT(5)
#define PMI8998_USBIN_PLUGIN_RT_STS			BIT(4)
#define PMI8998_USBIN_OV_RT_STS			BIT(3)
#define PMI8998_USBIN_UV_RT_STS			BIT(2)
#define PMI8998_USBIN_LT_3P6V_RT_STS		BIT(1)
#define PMI8998_USBIN_COLLAPSE_RT_STS		BIT(0)

#define PMI8998_QC_PULSE_COUNT_STATUS_1		0x30

#define PMI8998_USBIN_CMD_IL			0x40
#define PMI8998_BAT_2_SYS_FET_DIS			BIT(1)
#define PMI8998_USBIN_SUSPEND			BIT(0)

#define PMI8998_CMD_APSD				0x41
#define PMI8998_ICL_OVERRIDE			BIT(1)
#define PMI8998_APSD_RERUN				BIT(0)

#define PMI8998_CMD_HVDCP_2				0x43
#define PMI8998_RESTART_AICL			BIT(7)
#define PMI8998_TRIGGER_AICL			BIT(6)
#define PMI8998_FORCE_12V				BIT(5)
#define PMI8998_FORCE_9V				BIT(4)
#define PMI8998_FORCE_5V				BIT(3)
#define PMI8998_IDLE				BIT(2)
#define PMI8998_SINGLE_DECREMENT			BIT(1)
#define PMI8998_SINGLE_INCREMENT			BIT(0)

#define PMI8998_USB_MISC2				0x57
#define PMI8998_USB_MISC2_MASK				GENMASK(1, 0)

#define PMI8998_TYPE_C_CFG				0x58
#define PMI8998_APSD_START_ON_CC			BIT(7)
#define PMI8998_WAIT_FOR_APSD			BIT(6)
#define PMI8998_FACTORY_MODE_DETECTION_EN		BIT(5)
#define PMI8998_FACTORY_MODE_ICL_3A_4A		BIT(4)
#define PMI8998_FACTORY_MODE_DIS_CHGING_CFG		BIT(3)
#define PMI8998_SUSPEND_NON_COMPLIANT_CFG		BIT(2)
#define PMI8998_VCONN_OC_CFG			BIT(1)
#define PMI8998_TYPE_C_OR_U_USB			BIT(0)

#define PMI8998_TYPE_C_CFG_2			0x59
#define PMI8998_TYPE_C_DFP_CURRSRC_MODE		BIT(7)
#define PMI8998_DFP_CC_1P4V_OR_1P6V			BIT(6)
#define PMI8998_VCONN_SOFTSTART_CFG_MASK		GENMASK(5, 4)
#define PMI8998_EN_TRY_SOURCE_MODE			BIT(3)
#define PMI8998_USB_FACTORY_MODE_ENABLE		BIT(2)
#define PMI8998_TYPE_C_UFP_MODE			BIT(1)
#define PMI8998_EN_80UA_180UA_CUR_SOURCE		BIT(0)
#define TYPEC_SRC_RP_SEL_180UA 1
#define TYPEC_SRC_RP_SEL_80UA 0

#define PMI8998_TYPE_C_CFG_3			0x5A
#define PMI8998_TVBUS_DEBOUNCE			BIT(7)
#define PMI8998_TYPEC_LEGACY_CABLE_INT_EN		BIT(6)
#define PMI8998_TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN		BIT(5)
#define PMI8998_TYPEC_TRYSOURCE_DETECT_INT_EN	BIT(4)
#define PMI8998_TYPEC_TRYSINK_DETECT_INT_EN		BIT(3)
#define PMI8998_EN_TRYSINK_MODE			BIT(2)
#define PMI8998_EN_LEGACY_CABLE_DETECTION		BIT(1)
#define PMI8998_ALLOW_PD_DRING_UFP_TCCDB		BIT(0)

#define PMI8998_HVDCP_PULSE_COUNT_MAX		0x5B
#define PMI8998_HVDCP_PULSE_COUNT_MAX_QC2_MASK		GENMASK(7, 6)
enum {
	HVDCP_PULSE_COUNT_MAX_QC2_5V,
	HVDCP_PULSE_COUNT_MAX_QC2_9V,
	HVDCP_PULSE_COUNT_MAX_QC2_12V,
	HVDCP_PULSE_COUNT_MAX_QC2_INVALID
};

#define PMI8998_USBIN_ADAPTER_ALLOW_CFG		0x60
#define PMI8998_USBIN_ADAPTER_ALLOW_MASK		GENMASK(3, 0)
enum {
	USBIN_ADAPTER_ALLOW_5V		= 0,
	USBIN_ADAPTER_ALLOW_9V		= 2,
	USBIN_ADAPTER_ALLOW_5V_OR_9V	= 3,
	USBIN_ADAPTER_ALLOW_12V		= 4,
	USBIN_ADAPTER_ALLOW_5V_OR_12V	= 5,
	USBIN_ADAPTER_ALLOW_9V_TO_12V	= 6,
	USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V = 7,
	USBIN_ADAPTER_ALLOW_5V_TO_9V	= 8,
	USBIN_ADAPTER_ALLOW_5V_TO_12V	= 12,
};

#define PMI8998_USBIN_OPTIONS_1_CFG			0x62
#define PMI8998_CABLE_R_SEL				BIT(7)
#define PMI8998_HVDCP_AUTH_ALG_EN_CFG		BIT(6)
#define PMI8998_HVDCP_AUTONOMOUS_MODE_EN_CFG	BIT(5)
#define PMI8998_INPUT_PRIORITY			BIT(4)
#define PMI8998_AUTO_SRC_DETECT			BIT(3)
#define PMI8998_HVDCP_EN				BIT(2)
#define PMI8998_VADP_INCREMENT_VOLTAGE_LIMIT	BIT(1)
#define PMI8998_VADP_TAPER_TIMER_EN			BIT(0)

#define PMI8998_USBIN_OPTIONS_2_CFG			0x63
#define PMI8998_WIPWR_RST_EUD_CFG			BIT(7)
#define PMI8998_SWITCHER_START_CFG			BIT(6)
#define PMI8998_DCD_TIMEOUT_SEL			BIT(5)
#define PMI8998_OCD_CURRENT_SEL			BIT(4)
#define PMI8998_SLOW_PLUGIN_TIMER_EN_CFG		BIT(3)
#define PMI8998_FLOAT_OPTIONS_MASK			GENMASK(2, 0)
#define PMI8998_FLOAT_DIS_CHGING_CFG		BIT(2)
#define PMI8998_SUSPEND_FLOAT_CFG			BIT(1)
#define PMI8998_FORCE_FLOAT_SDP_CFG			BIT(0)

#define PMI8998_TAPER_TIMER_SEL_CFG			0x64
#define PMI8998_TYPEC_SPARE_CFG			BIT(7)
#define PMI8998_TYPEC_DRP_DFP_TIME_CFG		BIT(5)
#define PMI8998_TAPER_TIMER_SEL_MASK			GENMASK(1, 0)

#define PMI8998_USBIN_LOAD_CFG			0x65
#define PMI8998_USBIN_OV_CH_LOAD_OPTION		BIT(7)
#define PMI8998_ICL_OVERRIDE_AFTER_APSD		BIT(4)

#define PMI8998_USBIN_ICL_OPTIONS			0x66
#define PMI8998_CFG_USB3P0_SEL			BIT(2)
#define PMI8998_USB51_MODE				BIT(1)
#define PMI8998_USBIN_MODE_CHG			BIT(0)

#define PMI8998_TYPE_C_INTRPT_ENB			0x67
#define PMI8998_TYPEC_CCOUT_DETACH_INT_EN		BIT(7)
#define PMI8998_TYPEC_CCOUT_ATTACH_INT_EN		BIT(6)
#define PMI8998_TYPEC_VBUS_ERROR_INT_EN		BIT(5)
#define PMI8998_TYPEC_UFP_AUDIOADAPT_INT_EN		BIT(4)
#define PMI8998_TYPEC_DEBOUNCE_DONE_INT_EN		BIT(3)
#define PMI8998_TYPEC_CCSTATE_CHANGE_INT_EN		BIT(2)
#define PMI8998_TYPEC_VBUS_DEASSERT_INT_EN		BIT(1)
#define PMI8998_TYPEC_VBUS_ASSERT_INT_EN		BIT(0)

#define PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL	0x68
#define PMI8998_EXIT_SNK_BASED_ON_CC		BIT(7)
#define PMI8998_VCONN_EN_ORIENTATION		BIT(6)
#define PMI8998_TYPEC_VCONN_OVERCURR_INT_EN		BIT(5)
#define PMI8998_VCONN_EN_SRC			BIT(4)
#define PMI8998_VCONN_EN_VALUE			BIT(3)
#define PMI8998_PMI8998_TYPEC_POWER_ROLE_CMD_MASK	GENMASK(2, 1)
#define PMI8998_UFP_EN_CMD				BIT(2)
#define PMI8998_DFP_EN_CMD				BIT(1)
#define PMI8998_TYPEC_DISABLE_CMD			BIT(0)

#define PMI8998_USBIN_SOURCE_CHANGE_INTRPT_ENB	0x69
#define PMI8998_SLOW_IRQ_EN_CFG			BIT(5)
#define PMI8998_ENUMERATION_IRQ_EN_CFG		BIT(4)
#define PMI8998_VADP_IRQ_EN_CFG			BIT(3)
#define PMI8998_AUTH_IRQ_EN_CFG			BIT(2)
#define PMI8998_HVDCP_IRQ_EN_CFG			BIT(1)
#define PMI8998_APSD_IRQ_EN_CFG			BIT(0)

#define PMI8998_USBIN_CURRENT_LIMIT_CFG		0x70
#define PMI8998_USBIN_CURRENT_LIMIT_MASK		GENMASK(7, 0)

#define PMI8998_USBIN_AICL_OPTIONS_CFG		0x80
#define PMI8998_SUSPEND_ON_COLLAPSE_USBIN		BIT(7)
#define PMI8998_USBIN_AICL_HDC_EN			BIT(6)
#define PMI8998_USBIN_AICL_START_AT_MAX		BIT(5)
#define PMI8998_USBIN_AICL_RERUN_EN			BIT(4)
#define PMI8998_USBIN_AICL_ADC_EN			BIT(3)
#define PMI8998_USBIN_AICL_EN			BIT(2)
#define PMI8998_USBIN_HV_COLLAPSE_RESPONSE		BIT(1)
#define PMI8998_USBIN_LV_COLLAPSE_RESPONSE		BIT(0)

#define PMI8998_USBIN_5V_AICL_THRESHOLD_CFG		0x81
#define PMI8998_USBIN_5V_AICL_THRESHOLD_CFG_MASK	GENMASK(2, 0)

#define PMI8998_USBIN_9V_AICL_THRESHOLD_CFG		0x82
#define PMI8998_USBIN_9V_AICL_THRESHOLD_CFG_MASK	GENMASK(2, 0)

#define PMI8998_USBIN_12V_AICL_THRESHOLD_CFG	0x83
#define PMI8998_USBIN_12V_AICL_THRESHOLD_CFG_MASK	GENMASK(2, 0)

#define PMI8998_USBIN_CONT_AICL_THRESHOLD_CFG	0x84
#define PMI8998_USBIN_CONT_AICL_THRESHOLD_CFG_MASK	GENMASK(5, 0)

struct pmic_typec_port_irq_data {
	int				virq;
	int				irq;
	struct pmic_typec_port		*pmic_typec_port;
};

struct pmic_typec_port {
	struct device			*dev;
	struct tcpm_port		*tcpm_port;
	struct regmap			*regmap;
	u32				base;
	int irq;

	struct regulator		*vdd_vbus;
	bool				vbus_enabled;		/* We have set vbus to on */
	struct mutex			vbus_lock;		/* VBUS state serialization */

	int				cc;
	bool				debouncing_cc;
	bool				src_trywait;
	/* hardware reports vbus high since last irq */
	bool vbus_high;
	struct delayed_work		cc_debounce_dwork;

	spinlock_t			lock;	/* Register atomicity */
};

static const char * const typec_cc_status_name[] = {
	[TYPEC_CC_OPEN]		= "Open",
	[TYPEC_CC_RA]		= "Ra",
	[TYPEC_CC_RD]		= "Rd",
	[TYPEC_CC_RP_DEF]	= "Rp-def",
	[TYPEC_CC_RP_1_5]	= "Rp-1.5",
	[TYPEC_CC_RP_3_0]	= "Rp-3.0",
};

static const char *rp_unknown = "unknown";

static const char *cc_to_name(enum typec_cc_status cc)
{
	if (cc > TYPEC_CC_RP_3_0)
		return rp_unknown;

	return typec_cc_status_name[cc];
}

static const char * const rp_sel_name[] = {
	[TYPEC_SRC_RP_SEL_80UA]		= "Rp-def-80uA",
	[TYPEC_SRC_RP_SEL_180UA]	= "Rp-1.5-180uA",
};

static const char *rp_sel_to_name(int rp_sel)
{
	if (rp_sel > TYPEC_SRC_RP_SEL_180UA)
		return rp_unknown;

	return rp_sel_name[rp_sel];
}

#define misc_to_cc(msic) !!(misc & PMI8998_CC_ORIENTATION) ? "cc1" : "cc2"
#define misc_to_vconn(msic) !!(misc & PMI8998_CC_ORIENTATION) ? "cc2" : "cc1"

static inline int rp_sel_from_cc(enum typec_cc_status s)
{
	switch (s) {
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0: /* we only advertise up to 1.5A */
		return TYPEC_SRC_RP_SEL_180UA;
	case TYPEC_CC_RP_DEF:
	default:
		return TYPEC_SRC_RP_SEL_80UA;
	}
}

static inline unsigned int rp_bit_from_cc(enum typec_cc_status s)
{
	switch (s) {
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		return PMI8998_TYPE_C_DFP_CURRSRC_MODE; /* bit7 */
	default:
		return 0; /* Rp-def */
	}
}

static void qcom_pmic_typec_dbg_dump(struct pmic_typec_port *p, const char *tag)
{
	u32 s1 = 0, s2 = 0, s4 = 0, c2 = 0, c3 = 0, sw = 0, cmd = 0;

	/* Defensive: tolerate read errors without crashing the dump */
	regmap_read(p->regmap, p->base + PMI8998_TYPE_C_STATUS_1, &s1);
	regmap_read(p->regmap, p->base + PMI8998_TYPE_C_STATUS_2, &s2);
	regmap_read(p->regmap, p->base + PMI8998_TYPE_C_STATUS_4, &s4);
	regmap_read(p->regmap, p->base + PMI8998_TYPE_C_CFG_2, &c2);
	regmap_read(p->regmap, p->base + PMI8998_TYPE_C_CFG_3, &c3);
	regmap_read(p->regmap,
		    p->base + PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL, &sw);
	regmap_read(p->regmap, p->base + PMI8998_USBIN_CMD_IL, &cmd);

	/* Cast to u8 so the %02x formatting is correct/compact */
	dev_err(p->dev,
		"%s: C2=%02x C3=%02x SW=%02x CMD=%02x S1=%02x S2=%02x S4=%02x\n",
		tag ? tag : "dump", (u8)c2, (u8)c3, (u8)sw, (u8)cmd, (u8)s1,
		(u8)s2, (u8)s4);
}

static void qcom_pmic_typec_port_cc_debounce(struct work_struct *work)
{
	struct pmic_typec_port *pmic_typec_port =
		container_of(work, struct pmic_typec_port, cc_debounce_dwork.work);
	unsigned long flags;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);
	pmic_typec_port->debouncing_cc = false;
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(pmic_typec_port->dev, "Debounce cc complete\n");

	/*
	 * When in SRC_TRYWAIT the state machine will call set_cc and then expects
	 * a call to tcpm_cc_change() some time afterwards. However we never get
	 * an IRQ after this, so we just fake it here after debounce.
	 */
	if (pmic_typec_port->src_trywait) {
		tcpm_cc_change(pmic_typec_port->tcpm_port);
		pmic_typec_port->src_trywait = false;
	}
}

static irqreturn_t pmic_typec_port_isr(int irq, void *data)
{
	struct pmic_typec_port *pmic_typec_port = data;
	bool vbus_change = false;
	bool cc_change = false;
	unsigned long flags;
	int ret;
	u8 status[6];

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	qcom_pmic_typec_dbg_dump(pmic_typec_port, "pmic_typec_port_isr");

	ret = regmap_bulk_read(pmic_typec_port->regmap,
			       pmic_typec_port->base + PMI8998_TYPE_C_STATUS_1,
			       &status[1], ARRAY_SIZE(status)-1);
	if (ret)
		goto done;

	if (pmic_typec_port->vbus_high ^ !!(status[4] & PMI8998_TYPEC_VBUS_STATUS))
		vbus_change = true;

	cc_change = true;

done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	pmic_typec_port->vbus_high = !!(status[4] & PMI8998_TYPEC_VBUS_STATUS);

	if (vbus_change)
		tcpm_vbus_change(pmic_typec_port->tcpm_port);

	if (cc_change)
		tcpm_cc_change(pmic_typec_port->tcpm_port);

	return IRQ_HANDLED;
}

static int qcom_pmic_typec_port_vbus_detect(struct pmic_typec_port *pmic_typec_port)
{
	int ret;
	u32 misc;

	ret = regmap_read(pmic_typec_port->regmap,
			  pmic_typec_port->base + PMI8998_TYPE_C_STATUS_4,
			  &misc);
	if (ret) {
		dev_dbg(pmic_typec_port->dev, "Failed to read vbus: %d\n", ret);
		return false;
	}

	pmic_typec_port->vbus_high = !!(misc & PMI8998_TYPEC_VBUS_STATUS);

	return pmic_typec_port->vbus_high;
}

static int qcom_pmic_typec_port_vbus_toggle(struct pmic_typec_port *pmic_typec_port, bool on)
{
	int ret;

	if (on) {
		ret = regulator_enable(pmic_typec_port->vdd_vbus);
		if (ret)
			return ret;
	} else {
		ret = regulator_disable(pmic_typec_port->vdd_vbus);
		if (ret)
			return ret;
	}

	/* FIXME: regulator driver config! */
	msleep(10);

	return 0;
}

static int qcom_pmic_typec_port_get_vbus(struct tcpc_dev *tcpc)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	int ret;

	mutex_lock(&pmic_typec_port->vbus_lock);
	ret = pmic_typec_port->vbus_enabled || qcom_pmic_typec_port_vbus_detect(pmic_typec_port);
	mutex_unlock(&pmic_typec_port->vbus_lock);

	return ret;
}

static int qcom_pmic_typec_port_set_vbus(struct tcpc_dev *tcpc, bool on, bool sink)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	int ret = 0;

	mutex_lock(&pmic_typec_port->vbus_lock);
	if (pmic_typec_port->vbus_enabled == on)
		goto done;

	ret = qcom_pmic_typec_port_vbus_toggle(pmic_typec_port, on);
	if (ret)
		goto done;

	pmic_typec_port->vbus_enabled = on;
	tcpm_vbus_change(tcpm->tcpm_port);

done:
	dev_dbg(tcpm->dev, "set_vbus set: %d result %d\n", on, ret);
	mutex_unlock(&pmic_typec_port->vbus_lock);

	return ret;
}

static int qcom_pmic_typec_port_get_cc(struct tcpc_dev *tcpc,
				       enum typec_cc_status *cc1,
				       enum typec_cc_status *cc2)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	unsigned int misc, val;
	bool attached;
	int ret = 0;

	ret = regmap_read(pmic_typec_port->regmap,
			  pmic_typec_port->base + PMI8998_TYPE_C_STATUS_4,
			  &misc);
	if (ret)
		goto done;

	attached = !!(misc & PMI8998_CC_ATTACHED);

	if (pmic_typec_port->debouncing_cc) {
		ret = -EBUSY;
		goto done;
	}

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	if (!attached)
		goto done;

	if (misc & PMI8998_UFP_DFP_MODE_STATUS) {
		ret = regmap_read(pmic_typec_port->regmap,
				  pmic_typec_port->base + PMI8998_TYPE_C_STATUS_2,
				  &val);
		if (ret)
			goto done;
		switch (val & PMI8998_DFP_TYPEC_MASK) {
		// case AUDIO_ACCESS_RA_RA:
		// 	val = TYPEC_CC_RA;
		// 	*cc1 = TYPEC_CC_RA;
		// 	*cc2 = TYPEC_CC_RA;
		// 	break;
		case PMI8998_DFP_RD_OPEN:
			val = TYPEC_CC_RD;
			break;
		case PMI8998_DFP_RD_RA_VCONN:
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
		ret = regmap_read(pmic_typec_port->regmap,
				  pmic_typec_port->base + PMI8998_TYPE_C_STATUS_1,
				  &val);
		if (ret)
			goto done;
		switch (val & PMI8998_UFP_TYPEC_MASK) {
		case PMI8998_UFP_TYPEC_RDSTD:
			val = TYPEC_CC_RP_DEF;
			break;
		case PMI8998_UFP_TYPEC_RD1P5:
			val = TYPEC_CC_RP_1_5;
			break;
		case PMI8998_UFP_TYPEC_RD3P0:
			val = TYPEC_CC_RP_3_0;
			break;
		default:
			dev_warn(dev, "unexpected snk status %.2x\n", val);
			val = TYPEC_CC_RP_DEF;
			break;
		}
		val = TYPEC_CC_RP_DEF;
	}

	if (misc & PMI8998_CC_ORIENTATION)
		*cc2 = val;
	else
		*cc1 = val;

done:
	dev_dbg(dev, "get_cc: misc 0x%08x cc1 0x%08x %s cc2 0x%08x %s attached %d cc=%s\n",
		misc, *cc1, cc_to_name(*cc1), *cc2, cc_to_name(*cc2), attached,
		misc_to_cc(misc));

	return ret;
}

static void qcom_pmic_set_cc_debounce(struct pmic_typec_port *pmic_typec_port)
{
	pmic_typec_port->debouncing_cc = true;
	schedule_delayed_work(&pmic_typec_port->cc_debounce_dwork,
			      msecs_to_jiffies(2));
}

static int qcom_pmic_typec_port_set_cc(struct tcpc_dev *tcpc,
				       enum typec_cc_status cc)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	unsigned int mode, currsrc;
	unsigned int misc;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	ret = regmap_read(pmic_typec_port->regmap,
			  pmic_typec_port->base + PMI8998_TYPE_C_STATUS_4,
			  &misc);
	if (ret)
		goto done;

	mode = PMI8998_DFP_EN_CMD;

	switch (cc) {
	case TYPEC_CC_OPEN:
		currsrc = TYPEC_SRC_RP_SEL_80UA;
		break;
	case TYPEC_CC_RP_DEF:
		currsrc = TYPEC_SRC_RP_SEL_80UA;
		break;
	case TYPEC_CC_RP_1_5:
		currsrc = TYPEC_SRC_RP_SEL_180UA;
		break;
	case TYPEC_CC_RP_3_0:
		/* We can't source 3 amps, warn and lie... */
		dev_warn(dev, "unsupported Rp 3.0A, using 1.5A\n");
		currsrc = TYPEC_SRC_RP_SEL_180UA;
		break;
	case TYPEC_CC_RD:
		currsrc = TYPEC_SRC_RP_SEL_80UA;
		mode = PMI8998_UFP_EN_CMD;
		break;
	default:
		dev_warn(dev, "unexpected set_cc %d\n", cc);
		ret = -EINVAL;
		goto done;
	}

	if (mode == PMI8998_DFP_EN_CMD) {
		ret = regmap_write(pmic_typec_port->regmap,
				   pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
				   currsrc);
		if (ret)
			goto done;

		/* See debounce handler */
		pmic_typec_port->src_trywait = true;
	}

	pmic_typec_port->cc = cc;
	qcom_pmic_set_cc_debounce(pmic_typec_port);
	ret = 0;

done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(dev, "set_cc: currsrc=%x %s mode %s debounce %d attached %d cc=%s\n",
		currsrc, rp_sel_to_name(currsrc),
		mode == PMI8998_DFP_EN_CMD ? "EN_SRC_ONLY" : "EN_SNK_ONLY",
		pmic_typec_port->debouncing_cc, !!(misc & PMI8998_CC_ATTACHED),
		misc_to_cc(misc));

	return ret;
}

static int qcom_pmic_typec_port_set_polarity(struct tcpc_dev *tcpc,
					     enum typec_cc_polarity pol)
{
	/* Polarity is set separately by phy-qcom-qmp.c */
	return 0;
}

static int qcom_pmic_typec_port_set_vconn(struct tcpc_dev *tcpc, bool on)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	unsigned int orientation, misc, mask, value;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	ret = regmap_read(pmic_typec_port->regmap,
			  pmic_typec_port->base + PMI8998_TYPE_C_STATUS_4, &misc);
	if (ret)
		goto done;

	/* Set VCONN on the inversion of the active CC channel */
	orientation = (misc & PMI8998_CC_ORIENTATION) ? 0 : PMI8998_VCONN_EN_ORIENTATION;
	if (on) {
		mask = PMI8998_VCONN_EN_ORIENTATION | PMI8998_VCONN_EN_VALUE;
		value = orientation | PMI8998_VCONN_EN_VALUE | PMI8998_VCONN_EN_SRC;
	} else {
		mask = PMI8998_VCONN_EN_VALUE;
		value = 0;
	}

	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
				 mask, value);
done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(dev, "set_vconn: orientation %d control 0x%08x state %s cc %s vconn %s\n",
		orientation, value, on ? "on" : "off", misc_to_vconn(misc), misc_to_cc(misc));

	return ret;
}

#define TYPEC_INTR_ENB_MASK                   \
	(PMI8998_TYPEC_CCOUT_DETACH_INT_EN |  \
	 PMI8998_TYPEC_CCOUT_ATTACH_INT_EN |  \
	 PMI8998_TYPEC_VBUS_ASSERT_INT_EN |   \
	 PMI8998_TYPEC_VBUS_DEASSERT_INT_EN | \
	 PMI8998_TYPEC_VBUS_ERROR_INT_EN |    \
	 PMI8998_TYPEC_DEBOUNCE_DONE_INT_EN | \
	 PMI8998_TYPEC_CCSTATE_CHANGE_INT_EN)

static int qcom_pmic_typec_port_start_toggling(struct tcpc_dev *tcpc,
					       enum typec_port_type port_type,
					       enum typec_cc_status cc)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	unsigned long flags;
	unsigned int misc = 0;
	int ret = 0;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	/* Snapshot + dump before programming */
	regmap_read(pmic_typec_port->regmap,
		    pmic_typec_port->base + PMI8998_TYPE_C_STATUS_4, &misc);

	switch (port_type) {
	case TYPEC_PORT_SRC:
		/* Ensure DRP/Try.* are off */
		ret = regmap_update_bits(pmic_typec_port->regmap,
					 pmic_typec_port->base +
						 PMI8998_TYPE_C_CFG_2,
					 PMI8998_EN_TRY_SOURCE_MODE, 0);
		if (ret)
			break;

		ret = regmap_update_bits(pmic_typec_port->regmap,
					 pmic_typec_port->base +
						 PMI8998_TYPE_C_CFG_3,
					 PMI8998_EN_TRYSINK_MODE, 0);
		if (ret)
			break;

		/* Enable Rp current sources, pick advertised Rp (def/1.5) */
		ret = regmap_update_bits(pmic_typec_port->regmap,
					 pmic_typec_port->base +
						 PMI8998_TYPE_C_CFG_2,
					 PMI8998_EN_80UA_180UA_CUR_SOURCE,
					 PMI8998_EN_80UA_180UA_CUR_SOURCE);
		if (ret)
			break;

		// /* Select advertised Rp via bit7; keep rest of CFG_2 intact */
		// ret = regmap_update_bits(pmic_typec_port->regmap,
		// 	pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
		// 	PMI8998_TYPE_C_DFP_CURRSRC_MODE,
		// 	rp_bit_from_cc(cc));
		// if (ret) break;

		// /* Keep 1.6V threshold / tPDdebounce = 10–20ms */
		// regmap_update_bits(pmic_typec_port->regmap,
		// 	pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
		// 	PMI8998_DFP_CC_1P4V_OR_1P6V,
		// 	PMI8998_DFP_CC_1P4V_OR_1P6V);

		/* Force DFP (source) */
		ret = regmap_update_bits(
			pmic_typec_port->regmap,
			pmic_typec_port->base +
				PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
			PMI8998_UFP_EN_CMD | PMI8998_DFP_EN_CMD |
				PMI8998_TYPEC_DISABLE_CMD,
			PMI8998_DFP_EN_CMD);
		if (ret)
			break;

		dev_err(dev, "start_toggling: SRC, Rp=%d\n",
			rp_sel_from_cc(cc));
		break;

	case TYPEC_PORT_SNK:
		/* Ensure DRP/Try.* are off */
		ret = regmap_update_bits(pmic_typec_port->regmap,
					 pmic_typec_port->base +
						 PMI8998_TYPE_C_CFG_2,
					 PMI8998_EN_TRY_SOURCE_MODE, 0);
		if (ret)
			break;
		ret = regmap_update_bits(pmic_typec_port->regmap,
					 pmic_typec_port->base +
						 PMI8998_TYPE_C_CFG_3,
					 PMI8998_EN_TRYSINK_MODE, 0);
		if (ret)
			break;

		/* Optionally turn Rp current sources off while pure sink */
		regmap_update_bits(pmic_typec_port->regmap,
				   pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
				   PMI8998_EN_80UA_180UA_CUR_SOURCE, 0);

		/* Allow sinking current (don’t suspend USBIN) */
		regmap_update_bits(pmic_typec_port->regmap,
				   pmic_typec_port->base + PMI8998_USBIN_CMD_IL,
				   PMI8998_USBIN_SUSPEND, 0);

		/* Force UFP (sink) */
		ret = regmap_update_bits(
			pmic_typec_port->regmap,
			pmic_typec_port->base +
				PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
			PMI8998_UFP_EN_CMD | PMI8998_DFP_EN_CMD |
				PMI8998_TYPEC_DISABLE_CMD,
			PMI8998_UFP_EN_CMD);
		if (ret)
			break;

		dev_err(dev, "start_toggling: SNK\n");
		break;

	case TYPEC_PORT_DRP: {
		bool try_src;

		/* Bias DRP based on cc hint: Rp* -> Try.SRC, otherwise Try.SNK */
		switch (cc) {
		case TYPEC_CC_RP_DEF:
		case TYPEC_CC_RP_1_5:
		case TYPEC_CC_RP_3_0:
			try_src = true;
			break;
		default:
			try_src = false;
			break;
		}

		/* Clear any forced role first */
		ret = regmap_update_bits(
			pmic_typec_port->regmap,
			pmic_typec_port->base +
				PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
			PMI8998_UFP_EN_CMD | PMI8998_DFP_EN_CMD |
				PMI8998_TYPEC_DISABLE_CMD,
			0);
		if (ret)
			break;

		/* Enable useful Type-C IRQs */
		ret = regmap_update_bits(
			pmic_typec_port->regmap,
			pmic_typec_port->base + PMI8998_TYPE_C_INTRPT_ENB,
			TYPEC_INTR_ENB_MASK, TYPEC_INTR_ENB_MASK);
		if (ret)
			break;

		/* TVBUS debounce + Try.* detect IRQs + pick Try.SNK bit (if not Try.SRC) */
		ret = regmap_update_bits(
			pmic_typec_port->regmap,
			pmic_typec_port->base + PMI8998_TYPE_C_CFG_3,
			PMI8998_TVBUS_DEBOUNCE |
				PMI8998_TYPEC_TRYSOURCE_DETECT_INT_EN |
				PMI8998_TYPEC_TRYSINK_DETECT_INT_EN |
				PMI8998_EN_TRYSINK_MODE |
				PMI8998_EN_LEGACY_CABLE_DETECTION,
			PMI8998_TVBUS_DEBOUNCE |
				PMI8998_TYPEC_TRYSOURCE_DETECT_INT_EN |
				PMI8998_TYPEC_TRYSINK_DETECT_INT_EN |
				PMI8998_EN_TRYSINK_MODE /* <-- don’t clear this */);
		if (ret)
			break;

		/* Select Try.SRC (or clear it) + enable Rp sources + set CC threshold */
		ret = regmap_update_bits(
			pmic_typec_port->regmap,
			pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
			PMI8998_EN_TRY_SOURCE_MODE |
				PMI8998_EN_80UA_180UA_CUR_SOURCE |
				PMI8998_DFP_CC_1P4V_OR_1P6V,
			PMI8998_EN_TRY_SOURCE_MODE |
				PMI8998_EN_80UA_180UA_CUR_SOURCE |
				PMI8998_DFP_CC_1P4V_OR_1P6V);
		if (ret)
			break;

		/* Rp level we’ll advertise in the source half of DRP (bit7 only) */
		ret = regmap_update_bits(
			pmic_typec_port->regmap,
			pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
			PMI8998_TYPE_C_DFP_CURRSRC_MODE, rp_bit_from_cc(cc));
		if (ret)
			break;

		/* Ensure we can draw if we end up Sink */
		regmap_update_bits(pmic_typec_port->regmap,
				   pmic_typec_port->base + PMI8998_USBIN_CMD_IL,
				   PMI8998_USBIN_SUSPEND, 0);

		dev_dbg(dev, "start_toggling: DRP bias=%s (cc=%d)\n",
			try_src ? "Try.SRC" : "Try.SNK", cc);
		break;
	}
	}

	/* Keep your existing debounce behavior */
	qcom_pmic_set_cc_debounce(pmic_typec_port);

	pmic_typec_port->vbus_high = !!(misc & PMI8998_TYPEC_VBUS_STATUS);
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);
	return ret;
}

#define TYPEC_CFG_3_MASK \
	(PMI8998_TYPEC_LEGACY_CABLE_INT_EN | \
	PMI8998_TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN | \
	PMI8998_TYPEC_TRYSOURCE_DETECT_INT_EN | \
	PMI8998_TYPEC_TRYSINK_DETECT_INT_EN)

static int qcom_pmic_typec_port_start(struct pmic_typec *tcpm,
				      struct tcpm_port *tcpm_port)
{
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	int ret;

	/* 1) Enable Type-C/USB IRQs the driver relies on */
	ret = regmap_write(pmic_typec_port->regmap,
			   pmic_typec_port->base + PMI8998_TYPE_C_INTRPT_ENB,
			   TYPEC_INTR_ENB_MASK);
	if (ret)
		goto done;

	/* 2) Don’t force a role in SW; let DRP HW + Try.* take over */
	ret = regmap_update_bits(
		pmic_typec_port->regmap,
		pmic_typec_port->base + PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
		PMI8998_UFP_EN_CMD | PMI8998_DFP_EN_CMD |
			PMI8998_TYPEC_DISABLE_CMD,
		0);
	if (ret)
		goto done;

	/* 3) Configure DRP bias = Try.SRC.
     *    - Set EN_TRY_SOURCE_MODE in CFG_2
     *    - Make sure EN_TRYSINK_MODE is *cleared* in CFG_3
     *    - Keep TVBUS debounce and Try.* detect IRQ enables
     */
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
				 PMI8998_EN_TRY_SOURCE_MODE, /* mask */
				 PMI8998_EN_TRY_SOURCE_MODE); /* value */
	if (ret)
		goto done;

	/* Allow DRP Sink phase too (don’t block Try.SNK) + keep debounce & detect IRQs */
	ret = regmap_update_bits(
		pmic_typec_port->regmap,
		pmic_typec_port->base + PMI8998_TYPE_C_CFG_3,
		PMI8998_TVBUS_DEBOUNCE | PMI8998_TYPEC_TRYSOURCE_DETECT_INT_EN |
			PMI8998_TYPEC_TRYSINK_DETECT_INT_EN |
			PMI8998_EN_TRYSINK_MODE |
			PMI8998_EN_LEGACY_CABLE_DETECTION,
		PMI8998_TVBUS_DEBOUNCE | PMI8998_TYPEC_TRYSOURCE_DETECT_INT_EN |
			PMI8998_TYPEC_TRYSINK_DETECT_INT_EN |
			PMI8998_EN_TRYSINK_MODE /* <-- enable Try.SNK */);
	if (ret)
		goto done;

	/* 4) Advertise Rp when you’re the Source side of DRP */
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
				 PMI8998_EN_80UA_180UA_CUR_SOURCE,
				 PMI8998_EN_80UA_180UA_CUR_SOURCE);
	if (ret)
		goto done;

	/* 5) Keep CC threshold/tPDdebounce suitable for attach detection */
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + PMI8998_TYPE_C_CFG_2,
				 PMI8998_DFP_CC_1P4V_OR_1P6V,
				 PMI8998_DFP_CC_1P4V_OR_1P6V);
	if (ret)
		goto done;

	/* 6) Allow sinking if the attach race ends up as Sink (against fixed Rp).
     *    (If you *must* forbid sinking during Try.SRC, set USBIN_SUSPEND instead.)
     */
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + PMI8998_USBIN_CMD_IL,
				 PMI8998_USBIN_SUSPEND,
				 0); /* clear suspend -> sink path allowed */
	if (ret)
		goto done;

	/* 7) Put VCONN under SW control (we’ll switch it on when tcpm asks) */
	ret = regmap_write(pmic_typec_port->regmap,
			   pmic_typec_port->base +
				   PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
			   PMI8998_VCONN_EN_SRC);
	if (ret)
		goto done;

	/* 8) Disable APSD */
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base +
					 PMI8998_USBIN_OPTIONS_1_CFG,
				 PMI8998_AUTO_SRC_DETECT, 0);
	if (ret)
		goto done;

	/* 9) Disable HVDCP */
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base +
					 PMI8998_USBIN_OPTIONS_1_CFG,
				 PMI8998_HVDCP_EN, 0);
	if (ret)
		goto done;

	/* 10) SW controlled CC_OUT */
	ret = regmap_update_bits(
		pmic_typec_port->regmap,
		pmic_typec_port->base + PMI8998_TAPER_TIMER_SEL_CFG,
		PMI8998_TYPEC_SPARE_CFG, PMI8998_TYPEC_SPARE_CFG);
	if (ret)
		goto done;

	/* 11) Disable APSD on CC */
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + PMI8998_TYPE_C_CFG,
				 PMI8998_APSD_START_ON_CC, 0);
	if (ret)
		goto done;

	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + PMI8998_TYPE_C_CFG,
				 PMI8998_APSD_START_ON_CC, 0);
	if (ret)
		goto done;

	ret = regmap_write(pmic_typec_port->regmap, 0x1107, 0x00);
	if (ret)
		goto done;

	ret = regmap_write(pmic_typec_port->regmap, 0x1108, 0x00);
	if (ret)
		goto done;

	ret = regmap_write(pmic_typec_port->regmap, 0x1140, 0x00);
	if (ret)
		goto done;

	pmic_typec_port->tcpm_port = tcpm_port;
	enable_irq(pmic_typec_port->irq);
	return 0;

done:
	return ret;
}

static void qcom_pmic_typec_port_stop(struct pmic_typec *tcpm)
{
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;

	disable_irq(pmic_typec_port->irq);
}

int qcom_pmic_typec_port_probe_pmi8998(struct platform_device *pdev,
			       struct pmic_typec *tcpm,
			       const struct pmic_typec_port_resources *res,
			       struct regmap *regmap,
			       u32 base)
{
	struct device *dev = &pdev->dev;
	struct pmic_typec_port_irq_data *irq_data;
	struct pmic_typec_port *pmic_typec_port;
	int ret, irq;

	pmic_typec_port = devm_kzalloc(dev, sizeof(*pmic_typec_port), GFP_KERNEL);
	if (!pmic_typec_port)
		return -ENOMEM;

	if (!res->nr_irqs || res->nr_irqs > PMIC_TYPEC_MAX_IRQS)
		return -EINVAL;

	irq_data = devm_kzalloc(dev, sizeof(*irq_data) * res->nr_irqs,
				GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;

	mutex_init(&pmic_typec_port->vbus_lock);

	pmic_typec_port->vdd_vbus = devm_regulator_get(dev, "vdd-vbus");
	if (IS_ERR(pmic_typec_port->vdd_vbus))
		return PTR_ERR(pmic_typec_port->vdd_vbus);

	pmic_typec_port->dev = dev;
	pmic_typec_port->base = base;
	pmic_typec_port->regmap = regmap;
	spin_lock_init(&pmic_typec_port->lock);
	INIT_DELAYED_WORK(&pmic_typec_port->cc_debounce_dwork,
			  qcom_pmic_typec_port_cc_debounce);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL, pmic_typec_port_isr,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"type-c-change",
					pmic_typec_port);

	pmic_typec_port->irq = irq;
	tcpm->pmic_typec_port = pmic_typec_port;

	tcpm->tcpc.get_vbus = qcom_pmic_typec_port_get_vbus;
	tcpm->tcpc.set_vbus = qcom_pmic_typec_port_set_vbus;
	tcpm->tcpc.set_cc = qcom_pmic_typec_port_set_cc;
	tcpm->tcpc.get_cc = qcom_pmic_typec_port_get_cc;
	tcpm->tcpc.set_polarity = qcom_pmic_typec_port_set_polarity;
	tcpm->tcpc.set_vconn = qcom_pmic_typec_port_set_vconn;
	tcpm->tcpc.start_toggling = qcom_pmic_typec_port_start_toggling;

	tcpm->port_start = qcom_pmic_typec_port_start;
	tcpm->port_stop = qcom_pmic_typec_port_stop;

	return 0;
}
