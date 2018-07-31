/* LM3S6965 Ethernet Controller
 *
 * Copyright (c) 2018 Zilogic Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ETH_LM3S6965_PRIV_H_
#define ETH_LM3S6965_PRIV_H_

#define ETH_MTU 1500

#define DEV_DATA(dev) \
	((struct eth_lm3s6965_runtime *)(dev)->driver_data)
#define DEV_CFG(dev) \
	((const struct eth_lm3s6965_config *const)(dev)->config->config_info)
/*
 *  Register mapping
 */

/* Base address */
#define ETH_MAC_BASE_ADDR	0x40048000

/* Registers for ethernet system, mac_base + offset */
#define REG_MACRIS		((DEV_CFG(dev)->mac_base) + 0x000)
#define REG_MACIM		((DEV_CFG(dev)->mac_base) + 0x004)
#define REG_MACRCTL		((DEV_CFG(dev)->mac_base) + 0x008)
#define REG_MACTCTL		((DEV_CFG(dev)->mac_base) + 0x00c)
#define REG_MACDATA		((DEV_CFG(dev)->mac_base) + 0x010)
#define REG_MACIA0		((DEV_CFG(dev)->mac_base) + 0x014)
#define REG_MACIA1		((DEV_CFG(dev)->mac_base) + 0x018)
#define REG_MACTR		((DEV_CFG(dev)->mac_base) + 0x038)

/* ETH MAC Receive Control bit fields set value */
#define BIT_MACRCTL_RSTFIFO	0x10
#define BIT_MACRCTL_BADCRC	0x8
#define BIT_MACRCTL_RXEN	0x1
#define BIT_MACRCTL_PRMS	0x4

/* ETH MAC Transmit Control bit fields set value */
#define BIT_MACTCTL_DUPLEX	0x10
#define BIT_MACTCTL_CRC		0x4
#define BIT_MACTCTL_PADEN	0x2
#define BIT_MACTCTL_TXEN	0x1

/* ETH MAC Txn req bit fields set value */
#define BIT_MACTR_NEWTX		0x1

/* Ethernet MAC RAW Interrupt Status/Ack bit set values */
#define BIT_MACRIS_RXINT	0x1
#define BIT_MACRIS_TXER		0x2
#define BIT_MACRIS_TXEMP	0x4
#define BIT_MACRIS_RXER		0x16

struct eth_lm3s6965_runtime {
	struct net_if *iface;
	u8_t mac_addr[6];
	struct k_sem tx_sem;
	bool tx_err;
};

typedef void (*eth_lm3s6965_config_irq_t)(struct device *dev);

struct eth_lm3s6965_config {
	u32_t mac_base;
	u32_t sys_ctrl_base;
	u32_t irq_num;
	eth_lm3s6965_config_irq_t config_func;
};

#endif /* ETH_LM3S6965_PRIV_H_ */
