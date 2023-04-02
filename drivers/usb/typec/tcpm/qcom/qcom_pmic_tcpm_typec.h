/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021, Linaro Ltd. All rights reserved.
 */
#ifndef __QCOM_PMIC_TYPEC_H__
#define __QCOM_PMIC_TYPEC_H__

#include <linux/usb/tcpm.h>

struct pmic_typec;
extern struct platform_driver qcom_pmic_tcpm_typec_platform_driver;

int qcom_pmic_tcpm_typec_init(struct pmic_typec *pmic_typec,
			      struct tcpm_port *tcpm_port);

void qcom_pmic_tcpm_typec_put(struct pmic_typec *pmic_typec);

int qcom_pmic_tcpm_typec_get_cc(struct pmic_typec *pmic_typec,
				enum typec_cc_status *cc1,
				enum typec_cc_status *cc2);

int qcom_pmic_tcpm_typec_set_cc(struct pmic_typec *pmic_typec,
				enum typec_cc_status cc);

int qcom_pmic_tcpm_typec_get_vbus(struct pmic_typec *pmic_typec);

int qcom_pmic_tcpm_typec_set_current_limit(struct pmic_typec *pmic_typec,
                                          int ma);

int qcom_pmic_tcpm_typec_set_vconn(struct pmic_typec *pmic_typec, bool on);

int qcom_pmic_tcpm_typec_start_toggling(struct pmic_typec *pmic_typec,
					enum typec_port_type port_type,
					enum typec_cc_status cc);

int qcom_pmic_tcpm_typec_set_vbus(struct pmic_typec *pmic_typec, bool on);

#endif /* __QCOM_PMIC_TYPE_C_H__ */
