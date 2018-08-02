 /* STELLARIS Ethernet Controller
 *
 * Copyright (c) 2018 Zilogic Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_DOMAIN "dev/eth_stellaris"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_ETHERNET_LEVEL

#include <logging/sys_log.h>
#include <net/ethernet.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <device.h>
#include <soc.h>
#include <ethernet/eth_stats.h>
#include "eth_stellaris_priv.h"

static void eth_stellaris_assign_mac(struct device *dev)
{
	u32_t value = 0x0;

	value |= CONFIG_ETH_STELLARIS_MAC0;
	value |= CONFIG_ETH_STELLARIS_MAC1 << 8;
	value |= CONFIG_ETH_STELLARIS_MAC2 << 16;
	value |= CONFIG_ETH_STELLARIS_MAC3 << 24;
	sys_write32(value, REG_MACIA0);

	value = 0x0;
	value |= CONFIG_ETH_STELLARIS_MAC4;
	value |= CONFIG_ETH_STELLARIS_MAC5 << 8;
	sys_write32(value, REG_MACIA1);
}

static void eth_stellaris_dev_init(struct device *dev)
{
	u32_t value;

	ARG_UNUSED(dev);

	/* 1. Assign MAC address to Hardware */
	eth_stellaris_assign_mac(dev);

	/* 2. Program MCRCTL to clear RXFIFO */
	value = BIT_MACRCTL_RSTFIFO;
	sys_write32(value, REG_MACRCTL);

	/* 3. Enable transmitter */
	value = BIT_MACTCTL_DUPLEX | BIT_MACTCTL_CRC |
		BIT_MACTCTL_PADEN | BIT_MACTCTL_TXEN;
	sys_write32(value, REG_MACTCTL);

	/* 4. Enable Receiver */
	value = BIT_MACRCTL_BADCRC | BIT_MACRCTL_RXEN;
	sys_write32(value, REG_MACRCTL);
}

static int eth_stellaris_send(struct net_if *iface, struct net_pkt *pkt)
{
	struct net_eth_hdr *pkt_hdr;
	struct device *dev;
	struct eth_stellaris_runtime *dev_data;
	struct net_buf *frag;
	u32_t reg_val;
	u8_t bytes_left;
	int head_len_left;
	u8_t *data_ptr;

	dev = net_if_get_device(iface);
	dev_data = DEV_DATA(dev);

	if (!pkt->frags) {
		SYS_LOG_ERR("No data to send");
		return -ENODATA;
	}

	/* Frame transmission
	*  First two bytes are data_len for frame,
	*  and next two bytes are DST MAC(byte0 and 1).
	*  Initially transmit the ethernet header
	*/
	head_len_left = net_pkt_ll_reserve(pkt);
	data_ptr = net_pkt_ll(pkt);
	reg_val = net_pkt_get_len(pkt);
	reg_val |= ((u32_t)(*data_ptr++) << 16);
	reg_val |= ((u32_t)(*data_ptr++) << 24);

	/* Send the first word, part of header */
	sys_write32(reg_val, REG_MACDATA);
	head_len_left -= 2;

	/* Write the rest of header,
	 *  rest of header is 12 bytes and thus word aligned
	 */
	for (; head_len_left; head_len_left -= 4, data_ptr += 4) {
		sys_write32(*(u32_t *)data_ptr, REG_MACDATA);
	}

	/* Send the payload, if partial words are present,
	 * send them separately.
	 */
	for (frag = pkt->frags; frag; frag = frag->frags) {
		data_ptr = frag->data; bytes_left = frag->len;
		/* Check if partial words present */
		if ((bytes_left & 3) == 0) {
			/* No partial words present */
			for (; bytes_left > 0; bytes_left -= 4, data_ptr += 4) {
				sys_write32(*(u32_t *)data_ptr, REG_MACDATA);
			}
		} else {
			/* partial words present */
			for (; bytes_left > 3; bytes_left -= 4, data_ptr += 4) {
				sys_write32(*(u32_t *)data_ptr, REG_MACDATA);
			}
		}
	}

	/* Write the last, partial word in net_buf */
	if (bytes_left > 0) {
		reg_val = 0;
		switch (bytes_left) {
		case 3:
			reg_val |= ((u32_t)data_ptr[2] << 16);
		case 2:
			reg_val |= ((u32_t)data_ptr[1] << 8);
		case 1:
			reg_val |= (u32_t)data_ptr[0];
			break;
		default:
			break;
		}
		sys_write32(reg_val, REG_MACDATA);
	}

	/* Enable Txn */
	sys_write32(BIT_MACTR_NEWTX, REG_MACTR);

	/* Wait and check if txn successfull or not */
	k_sem_take(&dev_data->tx_sem, K_FOREVER);

	if (dev_data->tx_err) {
		SYS_LOG_ERR("Txn failed");
		dev_data->tx_err = false;
		return -EIO;
	}

	/* Update statistics counters */
	eth_stats_update_bytes_tx(iface, net_pkt_get_len(pkt));
	eth_stats_update_pkts_tx(iface);
	pkt_hdr = NET_ETH_HDR(pkt);
	if (net_eth_is_addr_multicast(&pkt_hdr->dst)) {
		eth_stats_update_multicast_tx(iface);
	} else if (net_eth_is_addr_broadcast(&pkt_hdr->dst)) {
		eth_stats_update_broadcast_tx(iface);
	}

	SYS_LOG_DBG("pkt send &p len &d", pkt, net_pkt_get_len(pkt));
	net_pkt_unref(pkt);

	return 0;
}

void eth_stellaris_rx(struct device *dev)
{
	struct net_pkt *pkt = NULL;
	struct eth_stellaris_runtime *dev_data = DEV_DATA(dev);
	struct net_if *iface = dev_data->iface;
	struct net_eth_hdr *pkt_hdr;
	u32_t reg_val;
	int pktlen, bytes_left, ret;

	/* Obtain the packet to be populated */
	pkt = net_pkt_get_reserve_rx(0, K_NO_WAIT);
	if (!pkt) {
		SYS_LOG_ERR("Could not allocate packet");
		eth_stats_update_errors_rx(iface);
		return;
	}

	/* Read the First word */
	reg_val = sys_read32(REG_MACDATA);
	pktlen = reg_val & 0x0000ffff;
	if (!net_pkt_append(pkt, 2, (u8_t *)&reg_val + 2, K_NO_WAIT)) {
		goto pkt_err;
	}

	/* Read the rest of words, minus the partial word and FCS byte */
	for (bytes_left = pktlen - 4; bytes_left > 7; bytes_left -= 4) {
		reg_val = sys_read32(REG_MACDATA);
		if (!net_pkt_append(pkt, 4, (u8_t *)&reg_val, K_NO_WAIT)) {
			goto pkt_err;
		}
	}

	/* Handle the last partial word and Discard the 4 Byte FCS */
	while (bytes_left > 0) {
		reg_val = sys_read32(REG_MACDATA);
		/* Discard the last FCS word */
		if (bytes_left <= 4) {
			bytes_left = 0;
			break;
		}
		/* Read the the partial word */
		if (!net_pkt_append(pkt, bytes_left - 4,
				    (u8_t *)&reg_val, K_NO_WAIT)) {
			goto pkt_err;
		}
		bytes_left -= 4;
	}

	/* Update statistics counters */
	eth_stats_update_bytes_rx(iface, pktlen);
	eth_stats_update_pkts_rx(iface);
	pkt_hdr = NET_ETH_HDR(pkt);
	if (net_eth_is_addr_broadcast(&pkt_hdr->dst)) {
		eth_stats_update_broadcast_rx(iface);
	} else if (net_eth_is_addr_multicast(&pkt_hdr->dst)) {
		eth_stats_update_multicast_rx(iface);
	}

	ret = net_recv_data(iface, pkt);
	if (ret < 0) {
		goto frame_err;
	}

	goto done;

frame_err:
	SYS_LOG_ERR("Failed to place frame in RX queue:%d", ret);
	goto update_error;

pkt_err:
	SYS_LOG_ERR("Failed to append data to buffer");

update_error:
	eth_stats_update_errors_rx(iface);
	net_pkt_unref(pkt);

done:	return;
}

static void rx_isr(void *arg)
{
	/* Read the interrupt status */
	struct device *dev = (struct device *)arg;
	struct eth_stellaris_runtime *dev_data = DEV_DATA(dev);
	int isr_val = sys_read32(REG_MACRIS);
	u32_t val;

	/* Acknowledge the interrupt */
	sys_write32(isr_val, REG_MACRIS);

	if (isr_val & BIT_MACRIS_RXINT) {

		eth_stellaris_rx(dev);

	} else if (isr_val & BIT_MACRIS_TXEMP) {

		k_sem_give(&dev_data->tx_sem);
		dev_data->tx_err = false;

	}  else if (isr_val & BIT_MACRIS_TXER) {

		eth_stats_update_errors_tx(dev_data->iface);
		dev_data->tx_err = true;
		k_sem_give(&dev_data->tx_sem);

	} else if (isr_val & BIT_MACRIS_RXER) {

		eth_stats_update_errors_rx(dev_data->iface);
		sys_write32(0x0, REG_MACRCTL);
		sys_write32(BIT_MACRCTL_RSTFIFO, REG_MACRCTL);
		val = BIT_MACRCTL_BADCRC | BIT_MACRCTL_RXEN;
		sys_write32(val, REG_MACRCTL);
	}
}

static void eth_stellaris_init(struct net_if *iface)
{
	struct device *dev;
	struct eth_stellaris_config *dev_conf;
	struct eth_stellaris_runtime *dev_data;

	dev = net_if_get_device(iface);
	dev_data = DEV_DATA(dev);
	dev_data->iface = iface;
	dev_conf = DEV_CFG(dev);

	/*1. Assign link local address */
	net_if_set_link_addr(iface,
			     dev_data->mac_addr, 6, NET_LINK_ETHERNET);

	/* Initialize semaphore */
	k_sem_init(&dev_data->tx_sem, 0, 1);

	/* Initialize Interrupts */
	dev_conf->config_func(dev);
}

static struct device DEVICE_NAME_GET(eth_stellaris);

static void eth_stellaris_irq_config(struct device *dev)
{
	ARG_UNUSED(dev);
	/* Enable Interrupt */
	IRQ_CONNECT(IRQ_ETH, 0, rx_isr, DEVICE_GET(eth_stellaris), 0);
	irq_enable(IRQ_ETH);
}

static struct eth_stellaris_config eth_cfg = {
	.mac_base = ETH_MAC_BASE_ADDR,
	.irq_num = IRQ_ETH,
	.config_func = eth_stellaris_irq_config,
};

static struct eth_stellaris_runtime eth_data = {
	.mac_addr = {
		CONFIG_ETH_STELLARIS_MAC0,
		CONFIG_ETH_STELLARIS_MAC1,
		CONFIG_ETH_STELLARIS_MAC2,
		CONFIG_ETH_STELLARIS_MAC3,
		CONFIG_ETH_STELLARIS_MAC4,
		CONFIG_ETH_STELLARIS_MAC5
	},
	.tx_err = false,
};

static const struct ethernet_api eth_stellaris_apis = {
	.iface_api.init	= eth_stellaris_init,
	.iface_api.send = eth_stellaris_send,
};

NET_DEVICE_INIT(eth_stellaris, CONFIG_ETH_STELLARIS_NAME,
		eth_stellaris_dev_init, &eth_data, &eth_cfg,
		CONFIG_ETH_INIT_PRIORITY,
		&eth_stellaris_apis, ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2), ETH_MTU);
