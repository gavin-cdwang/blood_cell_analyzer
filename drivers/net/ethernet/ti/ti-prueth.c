/*
 * PRU Ethernet Driver
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com
 *	Roger Quadros <rogerq@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/etherdevice.h>
#include <linux/genalloc.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/pruss.h>

#include "ti-prueth.h"
#include "ti-pru-mii.h"
#include "icss_switch.h"

#define PRUETH_MODULE_VERSION "0.1"
#define PRUETH_MODULE_DESCRIPTION "PRUSS Ethernet driver"

/* TX Minimum Inter packet gap */
#define TX_MIN_IPG		0xb8

#define TX_START_DELAY		0x40
#define TX_CLK_DELAY		0x6

#define IEP_GLOBAL_CFG_REG_VAL	0x0551

/* PRUSS local memory map */
#define ICSS_LOCAL_SHARED_RAM   0x00010000

/* Netif debug messages possible */
#define PRUETH_EMAC_DEBUG	(NETIF_MSG_DRV | \
				 NETIF_MSG_PROBE | \
				 NETIF_MSG_LINK | \
				 NETIF_MSG_TIMER | \
				 NETIF_MSG_IFDOWN | \
				 NETIF_MSG_IFUP | \
				 NETIF_MSG_RX_ERR | \
				 NETIF_MSG_TX_ERR | \
				 NETIF_MSG_TX_QUEUED | \
				 NETIF_MSG_INTR | \
				 NETIF_MSG_TX_DONE | \
				 NETIF_MSG_RX_STATUS | \
				 NETIF_MSG_PKTDATA | \
				 NETIF_MSG_HW | \
				 NETIF_MSG_WOL)

static int debug_level = -1;
module_param(debug_level, int, 0);
MODULE_PARM_DESC(debug_level, "PRUETH debug level (NETIF_MSG bits)");

#define EMAC_POLL_WEIGHT	(64) /* Default NAPI poll weight */
#define EMAC_MAX_PKTLEN		(ETH_HLEN + VLAN_HLEN + ETH_DATA_LEN)
#define EMAC_MIN_PKTLEN		(60)

/* In switch mode there are 3 real ports i.e. 3 mac addrs.
 * however Linux sees only the host side port. The other 2 ports
 * are the switch ports.
 * In emac mode there are 2 real ports i.e. 2 mac addrs.
 * Linux sees both the ports.
 */
enum prueth_port {
	PRUETH_PORT_HOST = 0,	/* host side port */
	PRUETH_PORT_MII0,	/* physical port MII 0 */
	PRUETH_PORT_MII1,	/* physical port MII 1 */
	PRUETH_PORT_MAX,
};

/* In both switch & emac modes there are 3 port queues
 * EMAC mode:
 *	RX packets for both MII0 & MII1 ports come on
 *	QUEUE_HOST.
 *	TX packets for MII0 go on QUEUE_MII0, TX packets
 *	for MII1 go on QUEUE_MII1.
 * Switch mode:
 *	Host port RX packets come on QUEUE_HOST
 *	TX packets might have to go on MII0 or MII1 or both.
 *	MII0 TX queue is QUEUE_MII0 and MII1 TX queue is
 *	QUEUE_MII1.
 */
enum prueth_port_queue_id {
	PRUETH_PORT_QUEUE_HOST = 0,
	PRUETH_PORT_QUEUE_MII0,
	PRUETH_PORT_QUEUE_MII1,
	PRUETH_PORT_QUEUE_MAX,
};

/* Each port queue has 4 queues and 1 collision queue */
enum prueth_queue_id {
	PRUETH_QUEUE1 = 0,
	PRUETH_QUEUE2,
	PRUETH_QUEUE3,
	PRUETH_QUEUE4,
	PRUETH_COLQUEUE,	/* collision queue */
};

/* PRUeth memory range identifiers */
enum prueth_mem {
	PRUETH_MEM_DRAM0 = 0,
	PRUETH_MEM_DRAM1,
	PRUETH_MEM_SHARED_RAM,
	PRUETH_MEM_IEP,
	PRUETH_MEM_MII,
	PRUETH_MEM_OCMC,
	PRUETH_MEM_MAX,
};

/* ensure that order of PRUSS mem regions is same as above */
static enum pruss_mem pruss_mem_ids[] = { PRUSS_MEM_DRAM0, PRUSS_MEM_DRAM1,
					  PRUSS_MEM_SHRD_RAM2, PRUSS_MEM_IEP,
					  PRUSS_MEM_MII_RT };

/**
 * struct prueth_match_data - match data to handle SoC integration
 * @pru_fw_names: array of firmware names to use for each PRU core
 */
struct prueth_match_data {
	const char *pru_fw_names[PRUSS_NUM_PRUS];
};

/* data for each emac port */
struct prueth_emac {
	struct prueth *prueth;
	struct net_device *ndev;
	u8 mac_addr[6];
	struct napi_struct napi;
	u32 msg_enable;

	int link;
	int speed;
	int duplex;

	const char *phy_id;
	struct device_node *phy_node;
	int phy_if;
	struct phy_device *phydev;

	enum prueth_port port_id;
	int rx_irq;
	int rx_sysevent;

	struct prueth_queue_desc __iomem *rx_queue_descs;
	struct prueth_queue_desc __iomem *tx_queue_descs;

	spinlock_t lock;	/* serialize access */
};

/**
 * struct prueth - PRUeth structure
 * @dev: device
 * @pruss: pruss handle
 * @pru0: rproc instance to PRU0
 * @pru1: rproc instance to PRU1
 * @data: device specific private data
 * @mem: PRUSS memory resources we need to access
 * @sram_pool: OCMC ram pool for buffers
 *
 * @eth_node: node for each emac node
 * @emac: emac data for three ports, one host and two physical
 * @registered_netdevs: net device for each registered emac
 */
struct prueth {
	struct device *dev;
	struct pruss *pruss;
	struct rproc *pru0, *pru1;
	const struct prueth_match_data *data;
	struct pruss_mem_region mem[PRUETH_MEM_MAX];
	struct gen_pool *sram_pool;

	struct device_node *eth_node[PRUETH_PORT_MAX];
	struct prueth_emac *emac[PRUETH_PORT_MAX];
	struct net_device *registered_netdevs[PRUETH_PORT_MAX];
};

static inline u32 prueth_read_reg(struct prueth *prueth,
				  enum prueth_mem region,
				  unsigned int reg)
{
	return readl_relaxed(prueth->mem[region].va + reg);
}

static inline void prueth_write_reg(struct prueth *prueth,
				    enum prueth_mem region,
				    unsigned int reg, u32 val)
{
	writel_relaxed(val, prueth->mem[region].va + reg);
}

static inline
void prueth_set_reg(struct prueth *prueth, enum prueth_mem region,
		    unsigned int reg, u32 mask, u32 set)
{
	u32 val;

	val = prueth_read_reg(prueth, region, reg);
	val &= ~mask;
	val |= (set & mask);
	prueth_write_reg(prueth, region, reg, val);
}

static const struct port_params port_params[] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		.queue = {
			[PRUETH_QUEUE1] = {
				.buffer_offset = P0_Q1_BUFFER_OFFSET,
				.buffer_desc_offset = P0_Q1_BD_OFFSET,
				.buffer_desc_count = HOST_QUEUE_1_SIZE,
			},
			[PRUETH_QUEUE2] = {
				.buffer_offset = P0_Q2_BUFFER_OFFSET,
				.buffer_desc_offset = P0_Q2_BD_OFFSET,
				.buffer_desc_count = HOST_QUEUE_2_SIZE,
			},
			[PRUETH_QUEUE3] = {
				.buffer_offset = P0_Q3_BUFFER_OFFSET,
				.buffer_desc_offset = P0_Q3_BD_OFFSET,
				.buffer_desc_count = HOST_QUEUE_3_SIZE,
			},
			[PRUETH_QUEUE4] = {
				.buffer_offset = P0_Q4_BUFFER_OFFSET,
				.buffer_desc_offset = P0_Q4_BD_OFFSET,
				.buffer_desc_count = HOST_QUEUE_4_SIZE,
			},
		},
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		.queue = {
			[PRUETH_QUEUE1] = {
				.buffer_offset = P1_Q1_BUFFER_OFFSET,
				.buffer_desc_offset = P1_Q1_BD_OFFSET,
				.buffer_desc_count = QUEUE_1_SIZE,
			},
			[PRUETH_QUEUE2] = {
				.buffer_offset = P1_Q2_BUFFER_OFFSET,
				.buffer_desc_offset = P1_Q2_BD_OFFSET,
				.buffer_desc_count = QUEUE_2_SIZE,
			},
			[PRUETH_QUEUE3] = {
				.buffer_offset = P1_Q3_BUFFER_OFFSET,
				.buffer_desc_offset = P1_Q3_BD_OFFSET,
				.buffer_desc_count = QUEUE_3_SIZE,
			},
			[PRUETH_QUEUE4] = {
				.buffer_offset = P1_Q4_BUFFER_OFFSET,
				.buffer_desc_offset = P1_Q4_BD_OFFSET,
				.buffer_desc_count = QUEUE_4_SIZE,
			},
		},
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		.queue = {
			[PRUETH_QUEUE1] = {
				.buffer_offset = P2_Q1_BUFFER_OFFSET,
				.buffer_desc_offset = P2_Q1_BD_OFFSET,
				.buffer_desc_count = QUEUE_1_SIZE,
			},
			[PRUETH_QUEUE2] = {
				.buffer_offset = P2_Q2_BUFFER_OFFSET,
				.buffer_desc_offset = P2_Q2_BD_OFFSET,
				.buffer_desc_count = QUEUE_2_SIZE,
			},
			[PRUETH_QUEUE3] = {
				.buffer_offset = P2_Q3_BUFFER_OFFSET,
				.buffer_desc_offset = P2_Q3_BD_OFFSET,
				.buffer_desc_count = QUEUE_3_SIZE,
			},
			[PRUETH_QUEUE4] = {
				.buffer_offset = P2_Q4_BUFFER_OFFSET,
				.buffer_desc_offset = P2_Q4_BD_OFFSET,
				.buffer_desc_count = QUEUE_4_SIZE,
			},
		},
	},
};

static const struct prueth_queue_info host_queue_info[] = {
	{
		P0_Q1_BUFFER_OFFSET,
		HOST_QUEUE_DESC_OFFSET,
		P0_Q1_BD_OFFSET,
		P0_Q1_BD_OFFSET + ((HOST_QUEUE_1_SIZE - 1) * BD_SIZE),
	},
	{
		P0_Q2_BUFFER_OFFSET,
		HOST_QUEUE_DESC_OFFSET + 8,
		P0_Q2_BD_OFFSET,
		P0_Q2_BD_OFFSET + ((HOST_QUEUE_2_SIZE - 1) * BD_SIZE),
	},
	{
		P0_Q3_BUFFER_OFFSET,
		HOST_QUEUE_DESC_OFFSET + 16,
		P0_Q3_BD_OFFSET,
		P0_Q3_BD_OFFSET + ((HOST_QUEUE_3_SIZE - 1) * BD_SIZE),
	},
	{
		P0_Q4_BUFFER_OFFSET,
		HOST_QUEUE_DESC_OFFSET + 24,
		P0_Q4_BD_OFFSET,
		P0_Q4_BD_OFFSET + ((HOST_QUEUE_4_SIZE - 1) * BD_SIZE),
	},
};

static const struct prueth_queue_desc host_queue_desc_init[] = {
	{ .rd_ptr = P0_Q1_BD_OFFSET, .wr_ptr = P0_Q1_BD_OFFSET, },
	{ .rd_ptr = P0_Q2_BD_OFFSET, .wr_ptr = P0_Q2_BD_OFFSET, },
	{ .rd_ptr = P0_Q3_BD_OFFSET, .wr_ptr = P0_Q3_BD_OFFSET, },
	{ .rd_ptr = P0_Q4_BD_OFFSET, .wr_ptr = P0_Q4_BD_OFFSET, },
};

static const struct prueth_queue_info p1_queue_info[] = {
	{
		P1_Q1_BUFFER_OFFSET,
		P1_Q1_BUFFER_OFFSET + ((QUEUE_1_SIZE - 1) * ICSS_BLOCK_SIZE),
		P1_Q1_BD_OFFSET,
		P1_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
	},
	{
		P1_Q2_BUFFER_OFFSET,
		P1_Q2_BUFFER_OFFSET + ((QUEUE_2_SIZE - 1) * ICSS_BLOCK_SIZE),
		P1_Q2_BD_OFFSET,
		P1_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
	},
	{
		P1_Q3_BUFFER_OFFSET,
		P1_Q3_BUFFER_OFFSET + ((QUEUE_3_SIZE - 1) * ICSS_BLOCK_SIZE),
		P1_Q3_BD_OFFSET,
		P1_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
	},
	{
		P1_Q4_BUFFER_OFFSET,
		P1_Q4_BUFFER_OFFSET + ((QUEUE_4_SIZE - 1) * ICSS_BLOCK_SIZE),
		P1_Q4_BD_OFFSET,
		P1_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
	},
};

static const struct prueth_queue_desc p1_queue_desc_init[] = {
	{ .rd_ptr = P1_Q1_BD_OFFSET, .wr_ptr = P1_Q1_BD_OFFSET, },
	{ .rd_ptr = P1_Q2_BD_OFFSET, .wr_ptr = P1_Q2_BD_OFFSET, },
	{ .rd_ptr = P1_Q3_BD_OFFSET, .wr_ptr = P1_Q3_BD_OFFSET, },
	{ .rd_ptr = P1_Q4_BD_OFFSET, .wr_ptr = P1_Q4_BD_OFFSET, },
};

static const struct prueth_queue_info p2_queue_info[] = {
	{
		P2_Q1_BUFFER_OFFSET,
		P2_Q1_BUFFER_OFFSET + ((QUEUE_1_SIZE - 1) * ICSS_BLOCK_SIZE),
		P2_Q1_BD_OFFSET,
		P2_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
	},
	{
		P2_Q2_BUFFER_OFFSET,
		P2_Q2_BUFFER_OFFSET + ((QUEUE_2_SIZE - 1) * ICSS_BLOCK_SIZE),
		P2_Q2_BD_OFFSET,
		P2_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
	},
	{
		P2_Q3_BUFFER_OFFSET,
		P2_Q3_BUFFER_OFFSET + ((QUEUE_3_SIZE - 1) * ICSS_BLOCK_SIZE),
		P2_Q3_BD_OFFSET,
		P2_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
	},
	{
		P2_Q4_BUFFER_OFFSET,
		P2_Q4_BUFFER_OFFSET + ((QUEUE_4_SIZE - 1) * ICSS_BLOCK_SIZE),
		P2_Q4_BD_OFFSET,
		P2_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
	},
};

static const struct prueth_queue_desc p2_queue_desc_init[] = {
	{ .rd_ptr = P2_Q1_BD_OFFSET, .wr_ptr = P2_Q1_BD_OFFSET, },
	{ .rd_ptr = P2_Q2_BD_OFFSET, .wr_ptr = P2_Q2_BD_OFFSET, },
	{ .rd_ptr = P2_Q3_BD_OFFSET, .wr_ptr = P2_Q3_BD_OFFSET, },
	{ .rd_ptr = P2_Q4_BD_OFFSET, .wr_ptr = P2_Q4_BD_OFFSET, },
};

/* uint8_t ICSSHostConfig(ICSSEMAC_Handle icssEmacHandle) */
static int prueth_hostconfig(struct prueth *prueth)
{
	void __iomem *sram_base = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	void __iomem *sram;

	/* queue size lookup table */
	sram = sram_base + QUEUE_SIZE_ADDR;
	writew(HOST_QUEUE_1_SIZE, sram);
	writew(HOST_QUEUE_2_SIZE, sram + 2);
	writew(HOST_QUEUE_3_SIZE, sram + 4);
	writew(HOST_QUEUE_4_SIZE, sram + 6);

	/* queue information table */
	sram = sram_base + HOST_Q1_RX_CONTEXT_OFFSET;
	memcpy_toio(sram, host_queue_info, sizeof(host_queue_info));

	/* buffer offset table */
	sram = sram_base + HOST_QUEUE_OFFSET_ADDR;
	writew(P0_Q1_BUFFER_OFFSET, sram);
	writew(P0_Q2_BUFFER_OFFSET, sram + 2);
	writew(P0_Q3_BUFFER_OFFSET, sram + 4);
	writew(P0_Q4_BUFFER_OFFSET, sram + 6);

	/* buffer descriptor offset table*/
	sram = sram_base + HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR;
	writew(P0_Q1_BD_OFFSET, sram);
	writew(P0_Q2_BD_OFFSET, sram + 2);
	writew(P0_Q3_BD_OFFSET, sram + 4);
	writew(P0_Q4_BD_OFFSET, sram + 6);

	/* queue table */
	sram = sram_base + HOST_QUEUE_DESC_OFFSET;
	memcpy_toio(sram, host_queue_desc_init, sizeof(host_queue_desc_init));

	return 0;
}

#define prueth_mii_set(reg, mask, set) \
	prueth_set_reg(prueth, PRUETH_MEM_MII, (reg), (mask), (set))

/* void PRUSSDRVPruMiiRtCfgInit(ICSSEMAC_Handle emacSubSysHandle) */
static void prueth_mii_init(struct prueth *prueth)
{
	/* Configuration of Port 0 Rx */
	prueth_mii_set(PRUSS_MII_RT_RXCFG0, PRUSS_MII_RT_RXCFG_RX_ENABLE, PRUSS_MII_RT_RXCFG_RX_ENABLE);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_RXCFG0, PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS, PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS);
	prueth_mii_set(PRUSS_MII_RT_RXCFG0, PRUSS_MII_RT_RXCFG_RX_MUX_SEL, 0x0);
	prueth_mii_set(PRUSS_MII_RT_RXCFG0, PRUSS_MII_RT_RXCFG_RX_L2_EN, PRUSS_MII_RT_RXCFG_RX_L2_EN);
	prueth_mii_set(PRUSS_MII_RT_RXCFG0, PRUSS_MII_RT_RXCFG_RX_CUT_PREAMBLE, PRUSS_MII_RT_RXCFG_RX_CUT_PREAMBLE);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_RXCFG0, PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS, PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS);

	/* Configuration of Port 0 Tx */
	prueth_mii_set(PRUSS_MII_RT_TXCFG0, PRUSS_MII_RT_TXCFG_TX_ENABLE, PRUSS_MII_RT_TXCFG_TX_ENABLE);
	prueth_mii_set(PRUSS_MII_RT_TXCFG0, PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE, PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_TXCFG0, PRUSS_MII_RT_TXCFG_TX_32_MODE_EN, PRUSS_MII_RT_TXCFG_TX_32_MODE_EN);
	prueth_mii_set(PRUSS_MII_RT_TXCFG0, PRUSS_MII_RT_TXCFG_TX_MUX_SEL, 0x0);
	prueth_mii_set(PRUSS_MII_RT_TXCFG0, PRUSS_MII_RT_TXCFG_TX_START_DELAY_MASK, TX_START_DELAY << PRUSS_MII_RT_TXCFG_TX_START_DELAY_SHIFT);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_TXCFG0, PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_MASK, TX_CLK_DELAY << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT);

	/* Configuration of Port 1 Rx */
	prueth_mii_set(PRUSS_MII_RT_RXCFG1, PRUSS_MII_RT_RXCFG_RX_ENABLE, PRUSS_MII_RT_RXCFG_RX_ENABLE);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_RXCFG1, PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS, PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS);
	prueth_mii_set(PRUSS_MII_RT_RXCFG1, PRUSS_MII_RT_RXCFG_RX_MUX_SEL, PRUSS_MII_RT_RXCFG_RX_MUX_SEL);
	prueth_mii_set(PRUSS_MII_RT_RXCFG1, PRUSS_MII_RT_RXCFG_RX_L2_EN, PRUSS_MII_RT_RXCFG_RX_L2_EN);
	prueth_mii_set(PRUSS_MII_RT_RXCFG1, PRUSS_MII_RT_RXCFG_RX_CUT_PREAMBLE, PRUSS_MII_RT_RXCFG_RX_CUT_PREAMBLE);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_RXCFG1, PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS, PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS);

	/* Configuration of Port 1 Tx */
	prueth_mii_set(PRUSS_MII_RT_TXCFG1, PRUSS_MII_RT_TXCFG_TX_ENABLE, PRUSS_MII_RT_TXCFG_TX_ENABLE);
	prueth_mii_set(PRUSS_MII_RT_TXCFG1, PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE, PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_TXCFG1, PRUSS_MII_RT_TXCFG_TX_32_MODE_EN, PRUSS_MII_RT_TXCFG_TX_32_MODE_EN);
	prueth_mii_set(PRUSS_MII_RT_TXCFG1, PRUSS_MII_RT_TXCFG_TX_MUX_SEL, PRUSS_MII_RT_TXCFG_TX_MUX_SEL);
	prueth_mii_set(PRUSS_MII_RT_TXCFG1, PRUSS_MII_RT_TXCFG_TX_START_DELAY_MASK, TX_START_DELAY << PRUSS_MII_RT_TXCFG_TX_START_DELAY_SHIFT);
	/* may not be needed on older silicon */
	prueth_mii_set(PRUSS_MII_RT_TXCFG1, PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_MASK, TX_CLK_DELAY << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT);
}

static void prueth_clearmem(struct prueth *prueth, enum prueth_mem region)
{
	memset_io(prueth->mem[region].va, 0, prueth->mem[region].size);
}

static int prueth_hostinit(struct prueth *prueth)
{
	/* Clear shared RAM */
	prueth_clearmem(prueth, PRUETH_MEM_SHARED_RAM);

	/* Clear OCMC RAM */
	prueth_clearmem(prueth, PRUETH_MEM_OCMC);

	prueth_hostconfig(prueth);

	prueth_mii_init(prueth);

	/* Enable IEP Counter */
	prueth_set_reg(prueth, PRUETH_MEM_IEP, 0, 0xffff,
		       IEP_GLOBAL_CFG_REG_VAL);

	return 0;
}

static int prueth_port_enable(struct prueth *prueth, enum prueth_port port,
			      bool enable)
{
	void __iomem *port_ctrl;

	if (port == PRUETH_PORT_MII0)
		port_ctrl = (prueth->mem[PRUETH_MEM_DRAM0].va +
			     PORT_CONTROL_ADDR);
	else if (port == PRUETH_PORT_MII1)
		port_ctrl = (prueth->mem[PRUETH_MEM_DRAM1].va +
			     PORT_CONTROL_ADDR);
	else
		return -EINVAL;

	if (enable)
		writeb(0x1, port_ctrl);
	else
		writeb(0x0, port_ctrl);

	return 0;
}

static int prueth_emac_config(struct prueth *prueth, struct prueth_emac *emac)
{
	/* PRU needs local shared RAM address for C28 */
	u32 sharedramaddr = ICSS_LOCAL_SHARED_RAM;
	/* PRU needs real global OCMC address for C30*/
	u32 ocmcaddr = (u32)prueth->mem[PRUETH_MEM_OCMC].pa;
	void __iomem *dram_base;
	void __iomem *mac_addr;
	void __iomem *dram;

	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		/* Clear data RAM */
		prueth_clearmem(prueth, PRUETH_MEM_DRAM0);

		dram_base = prueth->mem[PRUETH_MEM_DRAM0].va;

		/* setup mac address */
		mac_addr = dram_base + PORT_MAC_ADDR;
		memcpy_toio(mac_addr, emac->mac_addr, 6);

		/* queue information table */
		dram = dram_base + TX_CONTEXT_Q1_OFFSET_ADDR;
		memcpy_toio(dram, p1_queue_info, sizeof(p1_queue_info));

		/* queue table */
		dram = dram_base + PORT_QUEUE_DESC_OFFSET;
		memcpy_toio(dram, p1_queue_desc_init, sizeof(p1_queue_desc_init));

		/* Set in constant table C28 of PRU0 to ICSS Shared memory */
		pru_rproc_set_ctable(prueth->pru0, PRU_C28, sharedramaddr);
		/* Set in constant table C30 of PRU0 to OCMC memory */
		pru_rproc_set_ctable(prueth->pru0, PRU_C30, ocmcaddr);
		break;
	case PRUETH_PORT_MII1:
		/* Clear data RAM */
		prueth_clearmem(prueth, PRUETH_MEM_DRAM1);

		dram_base = prueth->mem[PRUETH_MEM_DRAM1].va;

		/* setup mac address */
		mac_addr = dram_base + PORT_MAC_ADDR;
		memcpy_toio(mac_addr, emac->mac_addr, 6);

		/* queue information table */
		dram = dram_base + TX_CONTEXT_Q1_OFFSET_ADDR;
		memcpy_toio(dram, p2_queue_info, sizeof(p2_queue_info));

		/* queue table */
		dram = dram_base + PORT_QUEUE_DESC_OFFSET;
		memcpy_toio(dram, p2_queue_desc_init, sizeof(p2_queue_desc_init));

		/* Set in constant table C28 of PRU1 to ICSS Shared memory */
		pru_rproc_set_ctable(prueth->pru1, PRU_C28, sharedramaddr);
		/* Set in constant table C30 of PRU1 to OCMC memory */
		pru_rproc_set_ctable(prueth->pru1, PRU_C30, ocmcaddr);
		break;
	default:
		netdev_err(emac->ndev, "invalid port\n");
		return -EINVAL;
	}

	return 0;
}

/* update phy/port status information for firmware */
static void emac_update_phystatus(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	enum prueth_mem region;
	u32 phy_speed, port_status = 0;

	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		region = PRUETH_MEM_DRAM0;
		break;
	case PRUETH_PORT_MII1:
		region = PRUETH_MEM_DRAM1;
		break;
	default:
		netdev_err(emac->ndev, "phy %s, invalid port\n",
			   dev_name(&emac->phydev->dev));
		return;
	}

	phy_speed = emac->speed;
	prueth_write_reg(prueth, region, PHY_SPEED_OFFSET, phy_speed);

	if (emac->duplex == DUPLEX_HALF)
		port_status |= PORT_IS_HD_MASK;
	if (emac->link)
		port_status |= PORT_LINK_MASK;
	writeb(port_status, prueth->mem[region].va + PORT_STATUS_OFFSET);
}

/* called back by PHY layer if there is change in link state of hw port*/
static void emac_adjust_link(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct phy_device *phydev = emac->phydev;
	unsigned long flags;
	bool new_state = false;

	spin_lock_irqsave(&emac->lock, flags);

	if (phydev->link) {
		/* check the mode of operation - full/half duplex */
		if (phydev->duplex != emac->duplex) {
			new_state = true;
			emac->duplex = phydev->duplex;
		}
		if (phydev->speed != emac->speed) {
			new_state = true;
			emac->speed = phydev->speed;
		}
		if (!emac->link) {
			new_state = true;
			emac->link = 1;
		}
	} else if (emac->link) {
		new_state = true;
		emac->link = 0;
		/* defaults for no link */
		emac->speed = SPEED_100; /* f/w only support 10 or 100 */
		emac->duplex = DUPLEX_FULL; /* half duplex may not be supported by f/w */
	}

	emac_update_phystatus(emac);

	if (new_state)
		phy_print_status(phydev);

	if (emac->link) {
		/* link ON */
		if (!netif_carrier_ok(ndev))
			netif_carrier_on(ndev);
		/* reactivate the transmit queue if it is stopped */
		if (netif_running(ndev) && netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
	} else {
		/* link OFF */
		if (netif_carrier_ok(ndev))
			netif_carrier_off(ndev);
		if (!netif_queue_stopped(ndev))
			netif_stop_queue(ndev);
	}

	spin_unlock_irqrestore(&emac->lock, flags);
}

/**
 * emac_rx_hardirq - EMAC Rx interrupt handler
 * @irq: interrupt number
 * @dev_id: pointer to net_device
 *
 * EMAC Interrupt handler - we only schedule NAPI and not process any packets
 * here.
 *
 * Returns interrupt handled condition
 */
static irqreturn_t emac_rx_hardirq(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	/* Check if the interrupt is for us */
	if (!pruss_intc_sysevent_check(prueth->pruss, emac->rx_sysevent))
		return IRQ_NONE;

	if (likely(netif_running(ndev))) {
		/* disable Rx system event */
		pruss_intc_sysevent_irqdisable(prueth->pruss,
					       emac->rx_sysevent);
		napi_schedule(&emac->napi);
	}

	/* Ack the interrupt */
	pruss_intc_sysevent_clear(prueth->pruss, emac->rx_sysevent);

	return IRQ_HANDLED;
}

/**
 * prueth_tx_enqueue - queue a packet to firmware for transmission
 *
 * @emac: EMAC data structure
 * @skb: packet data buffer
 * @txport: which port to send MII0 or MII1
 * @queue_id: priority queue id
 */
static int prueth_tx_enqueue(struct prueth_emac *emac, struct sk_buff *skb,
			     int txport, enum prueth_queue_id queue_id)
{
	struct net_device *ndev = emac->ndev;
	int pktlen;
	struct prueth_queue_desc __iomem *queue_desc;
	const struct queue *txqueue;
	u16 bd_rd_ptr, bd_wr_ptr, update_wr_ptr;
	int write_block, read_block, free_blocks, update_block, pkt_block_size;
	bool buffer_wrapped = false;
	void *src_addr;
	void *dst_addr;
	/* OCMC RAM is not cached and write order is not important */
	void *ocmc_ram = (__force void *)emac->prueth->mem[PRUETH_MEM_OCMC].va;
	void __iomem *dram;
	u32 wr_buf_desc;
	int ret;

	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		dram = emac->prueth->mem[PRUETH_MEM_DRAM0].va;
		break;
	case PRUETH_PORT_MII1:
		dram = emac->prueth->mem[PRUETH_MEM_DRAM1].va;
		break;
	default:
		netdev_err(emac->ndev, "invalid port\n");
		return -EINVAL;
	}

	ret = skb_padto(skb, EMAC_MIN_PKTLEN);
	if (ret) {
		if (netif_msg_tx_err(emac) && net_ratelimit())
			netdev_err(ndev, "packet pad failed");
		return ret;
	}
	src_addr = skb->data;

	/* pad packet if needed */
	pktlen = skb->len;
	if (pktlen < EMAC_MIN_PKTLEN)
		pktlen = EMAC_MIN_PKTLEN;

	/* Get the tx queue */
	queue_desc = emac->tx_queue_descs + queue_id;
	txqueue = &port_params[txport].queue[queue_id];

	bd_rd_ptr = readw(&queue_desc->rd_ptr);
	bd_wr_ptr = readw(&queue_desc->wr_ptr);

	/* the PRU firmware deals mostly in pointers already
	 * offset into ram, we would like to deal in indexes
	 * within the queue we are working with for code
	 * simplicity, calculate this here
	 */
	write_block = (bd_wr_ptr - txqueue->buffer_desc_offset) / BD_SIZE;
	read_block = (bd_rd_ptr - txqueue->buffer_desc_offset) / BD_SIZE;
	if (write_block > read_block) {
		free_blocks = txqueue->buffer_desc_count - write_block;
		free_blocks += read_block;
	} else if (write_block < read_block) {
		free_blocks = read_block - write_block;
	} else { /* they are all free */
		free_blocks = txqueue->buffer_desc_count;
	}
	pkt_block_size = DIV_ROUND_UP(pktlen, ICSS_BLOCK_SIZE);
	if (pkt_block_size > free_blocks) /* out of queue space */
		return -ENOBUFS;
	/* calculate end BD address post write */
	update_block = write_block + pkt_block_size;
	/* Check for wrap around */
	if (update_block >= txqueue->buffer_desc_count) {
		update_block %= txqueue->buffer_desc_count;
		buffer_wrapped = true;
	}

	dst_addr = ocmc_ram + txqueue->buffer_offset + (write_block * ICSS_BLOCK_SIZE);

	/* Copy the data from socket buffer(DRAM) to PRU buffers(OCMC) */
	if (buffer_wrapped) { /* wrapped around buffer */
		int bytes = (txqueue->buffer_desc_count - write_block) * ICSS_BLOCK_SIZE;
		int remaining;
		/* bytes is integral multiple of ICSS_BLOCK_SIZE but
		 * entire packet may have fit within the last BD
		 * if pkt_info.length is not integral multiple of
		 * ICSS_BLOCK_SIZE
		 */
		if (pktlen < bytes)
			bytes = pktlen;

		/* copy non-wrapped part */
		memcpy(dst_addr, src_addr, bytes);

		/* copy wrapped part */
		src_addr += bytes;
		remaining = pktlen - bytes;
		dst_addr = ocmc_ram + txqueue->buffer_offset;
		memcpy(dst_addr, src_addr, remaining);
	} else {
		memcpy(dst_addr, src_addr, pktlen);
	}

	/* update first buffer descriptor */
	wr_buf_desc = (pktlen << PRUETH_BD_LENGTH_SHIFT) & PRUETH_BD_LENGTH_MASK;
	writel(wr_buf_desc, dram + bd_wr_ptr);

	/* update the write pointer in this queue descriptor, the firmware
	 * polls for this change so this will signal the start of transmission
	 */
	update_wr_ptr = txqueue->buffer_desc_offset + (update_block * BD_SIZE);
	writew(update_wr_ptr, &queue_desc->wr_ptr);

	return 0;
}

static void parse_packet_info(u32 buffer_descriptor,
			      struct prueth_packet_info *pkt_info)
{
	pkt_info->shadow = !!(buffer_descriptor & PRUETH_BD_SHADOW_MASK);
	pkt_info->port = (buffer_descriptor & PRUETH_BD_PORT_MASK) >> PRUETH_BD_PORT_SHIFT;
	pkt_info->length = (buffer_descriptor & PRUETH_BD_LENGTH_MASK) >> PRUETH_BD_LENGTH_SHIFT;
	pkt_info->broadcast = !!(buffer_descriptor & PRUETH_BD_BROADCAST_MASK);
	pkt_info->error = !!(buffer_descriptor & PRUETH_BD_ERROR_MASK);
}

/* get packet from queue
 * negative for error
 */
static int emac_rx_packet(struct prueth_emac *emac, u16 *bd_rd_ptr,
			  struct prueth_packet_info pkt_info,
			  const struct queue *rxqueue)
{
	struct net_device *ndev = emac->ndev;
	int read_block, update_block, pkt_block_size;
	bool buffer_wrapped = false;
	struct sk_buff *skb;
	void *src_addr;
	void *dst_addr;
	/* OCMC RAM is not cached and read order is not important */
	void *ocmc_ram = (__force void *)emac->prueth->mem[PRUETH_MEM_OCMC].va;

	/* the PRU firmware deals mostly in pointers already
	 * offset into ram, we would like to deal in indexes
	 * within the queue we are working with for code
	 * simplicity, calculate this here
	 */
	read_block = (*bd_rd_ptr - rxqueue->buffer_desc_offset) / BD_SIZE;
	pkt_block_size = DIV_ROUND_UP(pkt_info.length, ICSS_BLOCK_SIZE);
	/* calculate end BD address post read */
	update_block = read_block + pkt_block_size;
	/* Check for wrap around */
	if (update_block >= rxqueue->buffer_desc_count) {
		update_block %= rxqueue->buffer_desc_count;
		buffer_wrapped = true;
	}

	/* calculate new pointer in ram */
	*bd_rd_ptr = rxqueue->buffer_desc_offset + (update_block * BD_SIZE);

	/* Allocate a socket buffer for this packet */
	skb = netdev_alloc_skb_ip_align(ndev, pkt_info.length);
	if (!skb) {
		if (netif_msg_rx_err(emac) && net_ratelimit())
			netdev_err(ndev, "failed rx buffer alloc\n");
		return -ENOMEM;
	}
	dst_addr = skb->data;

	/* Get the start address of the first buffer from
	 * the read buffer description
	 */
	if (pkt_info.shadow)
		/* Pick the data from collision buffer */
		src_addr = ocmc_ram + P0_COL_BUFFER_OFFSET;
	else
		src_addr = ocmc_ram + rxqueue->buffer_offset +
				(read_block * ICSS_BLOCK_SIZE);

	/* Copy the data from PRU buffers(OCMC) to socket buffer(DRAM) */
	if (buffer_wrapped) { /* wrapped around buffer */
		int bytes = (rxqueue->buffer_desc_count - read_block) * ICSS_BLOCK_SIZE;
		int remaining;
		/* bytes is integral multiple of ICSS_BLOCK_SIZE but
		 * entire packet may have fit within the last BD
		 * if pkt_info.length is not integral multiple of
		 * ICSS_BLOCK_SIZE
		 */
		if (pkt_info.length < bytes)
			bytes = pkt_info.length;

		/* copy non-wrapped part */
		memcpy(dst_addr, src_addr, bytes);

		/* copy wrapped part */
		dst_addr += bytes;
		remaining = pkt_info.length - bytes;
		if (pkt_info.shadow)
			src_addr = src_addr + bytes;
		else
			src_addr = ocmc_ram + rxqueue->buffer_offset;
		memcpy(dst_addr, src_addr, remaining);
	} else {
		memcpy(dst_addr, src_addr, pkt_info.length);
	}

	/* send packet up the stack */
	skb_put(skb, pkt_info.length);
	skb->protocol = eth_type_trans(skb, ndev);
	netif_receive_skb(skb);

	/* update stats */
	ndev->stats.rx_bytes += pkt_info.length;
	ndev->stats.rx_packets++;

	return 0;
}

/* get upto quota number of packets */
static int emac_rx_packets(struct prueth_emac *emac, int quota)
{
	int start_queue, end_queue;
	struct prueth_queue_desc __iomem *queue_desc;
	const struct queue *rxqueue;
	u8 overflow_cnt;
	u16 bd_rd_ptr, bd_wr_ptr, update_rd_ptr;
	u32 rd_buf_desc;
	void __iomem *shared_ram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	struct prueth_packet_info pkt_info;
	struct net_device_stats *ndevstats = &emac->ndev->stats;
	int i, ret, used = 0;

	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		/* packets from MII0 are on queues 1 through 2 */
		start_queue = PRUETH_QUEUE2;
		end_queue = PRUETH_QUEUE1;
		break;
	case PRUETH_PORT_MII1:
		/* packets from MII1 are on queues 3 through 4 */
		start_queue = PRUETH_QUEUE4;
		end_queue = PRUETH_QUEUE3;
		break;
	default:
		netdev_err(emac->ndev, "invalid port\n");
		return -EINVAL;
	}

	/* search host queues for packets */
	for (i = start_queue; i >= end_queue; i--) {
		queue_desc = emac->rx_queue_descs + i;
		rxqueue = &port_params[PRUETH_PORT_HOST].queue[i];

		overflow_cnt = readb(&queue_desc->overflow_cnt);
		if (overflow_cnt > 0) {
			emac->ndev->stats.rx_over_errors += overflow_cnt;
			/* reset to zero */
			writeb(0, &queue_desc->overflow_cnt);
		}

		bd_rd_ptr = readw(&queue_desc->rd_ptr);
		bd_wr_ptr = readw(&queue_desc->wr_ptr);

		/* while packets are available in this queue */
		while (bd_rd_ptr != bd_wr_ptr) {
			/* get packet info from the read buffer descriptor */
			rd_buf_desc = readl(shared_ram + bd_rd_ptr);
			parse_packet_info(rd_buf_desc, &pkt_info);

			if (pkt_info.length <= 0) {
				/* a packet length of zero will cause us to
				 * never move the read pointer ahead, locking
				 * the driver, so we manually have to move it
				 * to the write pointer, discarding all
				 * remaining packets in this queue. This should
				 * never happen.
				 */
				update_rd_ptr = bd_wr_ptr;
				ndevstats->rx_length_errors++;
			} else if (pkt_info.length > EMAC_MAX_PKTLEN) {
				/* if the packet is too large we skip it but we
				 * still need to move the read pointer ahead
				 * and assume something is wrong with the read
				 * pointer as the firmware should be filtering
				 * these packets
				 */
				update_rd_ptr = bd_wr_ptr;
				ndevstats->rx_length_errors++;
			} else {
				update_rd_ptr = bd_rd_ptr;
				ret = emac_rx_packet(emac, &update_rd_ptr,
						     pkt_info, rxqueue);
				if (ret)
					return ret;

				used++;
			}

			/* after reading the buffer descriptor we clear it
			 * to prevent improperly moved read pointer errors
			 * from simply looking like old packets.
			 */
			writel(0, shared_ram + bd_rd_ptr);

			/* update read pointer in queue descriptor */
			writew(update_rd_ptr, &queue_desc->rd_ptr);
			bd_rd_ptr = update_rd_ptr;

			/* all we have room for? */
			if (used >= quota)
				return used;
		}
	}

	return used;
}

/* get statistics maintained by the PRU firmware into @pstats */
static void emac_get_stats(struct prueth_emac *emac,
			   struct port_statistics *pstats)
{
	void __iomem *dram;

	if (emac->port_id == PRUETH_PORT_MII0)
		dram = emac->prueth->mem[PRUETH_MEM_DRAM0].va;
	else
		dram = emac->prueth->mem[PRUETH_MEM_DRAM1].va;

	memcpy_fromio(pstats, dram + STATISTICS_OFFSET, sizeof(*pstats));
}

/**
 * emac_napi_poll - EMAC NAPI Poll function
 * @ndev: EMAC network adapter
 * @budget: Number of receive packets to process (as told by NAPI layer)
 *
 * NAPI Poll function implemented to process packets as per budget. We check
 * the type of interrupt on the device and accordingly call the TX or RX
 * packet processing functions. We follow the budget for RX processing and
 * also put a cap on number of TX pkts processed through config param. The
 * NAPI schedule function is called if more packets pending.
 *
 * Returns number of packets received (in most cases; else TX pkts - rarely)
 */
static int emac_napi_poll(struct napi_struct *napi, int budget)
{
	struct prueth_emac *emac = container_of(napi, struct prueth_emac, napi);
	int num_rx_packets;

	num_rx_packets = emac_rx_packets(emac, budget);
	if (num_rx_packets < budget) {
		napi_complete(napi);

		pruss_intc_sysevent_irqenable(emac->prueth->pruss,
					      emac->rx_sysevent);
	}

	return num_rx_packets;
}

/**
 * emac_ndo_open - EMAC device open
 * @ndev: network adapter device
 *
 * Called when system wants to start the interface.
 *
 * Returns 0 for a successful open, or appropriate error code
 */
static int emac_ndo_open(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct pruss *pruss = prueth->pruss;
	int ret;

	ret = request_irq(emac->rx_irq, emac_rx_hardirq,
			  IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			  ndev->name, ndev);
	if (ret) {
		netdev_err(ndev, "failed to request irq %d\n", emac->rx_irq);
		return ret;
	}

	/* set h/w MAC as user might have re-configured */
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	netif_carrier_off(ndev);

	/* enable Rx system event */
	pruss_intc_sysevent_irqenable(pruss, emac->rx_sysevent);

	/* reset and start PRU firmware */
	prueth_emac_config(prueth, emac);
	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		ret = pruss_rproc_boot(pruss, prueth->pru0,
				       prueth->data->pru_fw_names[PRUSS_PRU0]);
		if (ret) {
			netdev_err(ndev, "failed to boot PRU0: %d\n", ret);
			goto free_irq;
		}
		break;
	case PRUETH_PORT_MII1:
		ret = pruss_rproc_boot(pruss, prueth->pru1,
				       prueth->data->pru_fw_names[PRUSS_PRU1]);
		if (ret) {
			netdev_err(ndev, "failed to boot PRU1: %d\n", ret);
			goto free_irq;
		}
		break;
	default:
		/* switch mode not supported yet */
		netdev_err(ndev, "invalid port\n");
		goto free_irq;
	}

	/* start PHY */
	phy_start(emac->phydev);

	napi_enable(&emac->napi);

	/* enable the port */
	prueth_port_enable(prueth, emac->port_id, true);

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "started\n");

	return 0;

free_irq:
	free_irq(emac->rx_irq, ndev);

	return ret;
}

/**
 * emac_ndo_stop - EMAC device stop
 * @ndev: network adapter device
 *
 * Called when system wants to stop or down the interface.
 */
static int emac_ndo_stop(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct pruss *pruss = prueth->pruss;

	/* inform the upper layers. */
	netif_stop_queue(ndev);
	napi_disable(&emac->napi);
	netif_carrier_off(ndev);

	/* stop PHY */
	phy_stop(emac->phydev);

	/* disable the mac port */
	prueth_port_enable(emac->prueth, emac->port_id, 0);

	/* stop PRU firmware */
	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		pruss_rproc_halt(pruss, prueth->pru0);
		break;
	case PRUETH_PORT_MII1:
		pruss_rproc_halt(pruss, prueth->pru1);
		break;
	default:
		/* switch mode not supported yet */
		netdev_err(ndev, "invalid port\n");
	}

	/* disable Rx system event */
	pruss_intc_sysevent_irqdisable(emac->prueth->pruss,
				       emac->rx_sysevent);
	free_irq(emac->rx_irq, ndev);

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "stopped\n");

	return 0;
}

/**
 * emac_ndo_start_xmit - EMAC Transmit function
 * @skb: SKB pointer
 * @ndev: EMAC network adapter
 *
 * Called by the system to transmit a packet  - we queue the packet in
 * EMAC hardware transmit queue
 *
 * Returns success(NETDEV_TX_OK) or error code (typically out of desc's)
 */
static int emac_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int ret = 0;

	if (unlikely(!emac->link)) {
		if (netif_msg_tx_err(emac) && net_ratelimit())
			netdev_err(ndev, "No link to transmit");
		goto fail_tx;
	}

	if (emac->port_id == PRUETH_PORT_MII0) {
		/* packet sent on MII0 */
		ret = prueth_tx_enqueue(emac, skb, PRUETH_PORT_QUEUE_MII0,
					PRUETH_QUEUE4);
	} else if (emac->port_id == PRUETH_PORT_MII1) {
		/* packet sent on MII1 */
		ret = prueth_tx_enqueue(emac, skb, PRUETH_PORT_QUEUE_MII1,
					PRUETH_QUEUE4);
	} else {
		goto fail_tx; /* switch mode not supported yet */
	}

	if (ret) {
		if (ret != -ENOBUFS && netif_msg_tx_err(emac) && net_ratelimit())
			netdev_err(ndev, "packet queue failed: %d\n", ret);
		goto fail_tx;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;

fail_tx:
	/* error */
	ndev->stats.tx_dropped++;

	return NETDEV_TX_BUSY;
}

/**
 * emac_ndo_tx_timeout - EMAC Transmit timeout function
 * @ndev: The EMAC network adapter
 *
 * Called when system detects that a skb timeout period has expired
 * potentially due to a fault in the adapter in not being able to send
 * it out on the wire.
 */
static void emac_ndo_tx_timeout(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (netif_msg_tx_err(emac))
		netdev_err(ndev, "xmit timeout");

	ndev->stats.tx_errors++;

	/* TODO: can we recover or need to reboot firmware? */
}

/**
 * emac_ndo_getstats - EMAC get statistics function
 * @ndev: The EMAC network adapter
 *
 * Called when system wants to get statistics from the device.
 *
 * We return the statistics in net_device_stats structure pulled from emac
 */
static struct net_device_stats *emac_ndo_get_stats(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct port_statistics pstats;
	struct net_device_stats *stats = &ndev->stats;

	emac_get_stats(emac, &pstats);
	stats->collisions = pstats.late_coll + pstats.single_coll +
			    pstats.multi_coll + pstats.excess_coll;
	stats->multicast = pstats.rx_mcast;

	return stats;
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open = emac_ndo_open,
	.ndo_stop = emac_ndo_stop,
	.ndo_start_xmit = emac_ndo_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_mtu	= eth_change_mtu,
	.ndo_tx_timeout = emac_ndo_tx_timeout,
	.ndo_get_stats = emac_ndo_get_stats,
};

/**
 * emac_get_drvinfo - Get EMAC driver information
 * @ndev: The network adapter
 * @info: ethtool info structure containing name and version
 *
 * Returns EMAC driver information (name and version)
 */
static void emac_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, PRUETH_MODULE_DESCRIPTION, sizeof(info->driver));
	strlcpy(info->version, PRUETH_MODULE_VERSION, sizeof(info->version));
}

/**
 * emac_get_settings - Get EMAC settings
 * @ndev: The network adapter
 * @ecmd: ethtool command
 *
 * Executes ethool get command
 */
static int emac_get_settings(struct net_device *ndev, struct ethtool_cmd *ecmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (emac->phydev)
		return phy_ethtool_gset(emac->phydev, ecmd);
	else
		return -EOPNOTSUPP;
}

/**
 * emac_set_settings - Set EMAC settings
 * @ndev: The EMAC network adapter
 * @ecmd: ethtool command
 *
 * Executes ethool set command
 */
static int emac_set_settings(struct net_device *ndev, struct ethtool_cmd *ecmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (emac->phydev)
		return phy_ethtool_sset(emac->phydev, ecmd);
	else
		return -EOPNOTSUPP;
}

#define PRUETH_STAT_OFFSET(m) offsetof(struct port_statistics, m)

static const struct {
	char string[ETH_GSTRING_LEN];
	u32 offset;
} prueth_ethtool_stats[] = {
	{"txBcast", PRUETH_STAT_OFFSET(tx_bcast)},
	{"txMcast", PRUETH_STAT_OFFSET(tx_mcast)},
	{"txUcast", PRUETH_STAT_OFFSET(tx_ucast)},
	{"txOctets", PRUETH_STAT_OFFSET(tx_octets)},
	{"rxBcast", PRUETH_STAT_OFFSET(rx_bcast)},
	{"rxMcast", PRUETH_STAT_OFFSET(rx_mcast)},
	{"rxUcast", PRUETH_STAT_OFFSET(rx_ucast)},
	{"rxOctets", PRUETH_STAT_OFFSET(rx_octets)},

	{"lateColl", PRUETH_STAT_OFFSET(late_coll)},
	{"singleColl", PRUETH_STAT_OFFSET(single_coll)},
	{"multiColl", PRUETH_STAT_OFFSET(multi_coll)},
	{"excessColl", PRUETH_STAT_OFFSET(excess_coll)},

	{"txHWQOverFlow", PRUETH_STAT_OFFSET(tx_hwq_overflow)},
	{"rxMisAlignmentFrames", PRUETH_STAT_OFFSET(rx_misalignment_frames)},
	{"stormPrevCounter", PRUETH_STAT_OFFSET(stormprev_counter)},
	{"macRxError", PRUETH_STAT_OFFSET(mac_rxerror)},
	{"SFDError", PRUETH_STAT_OFFSET(sfd_error)},
	{"defTx", PRUETH_STAT_OFFSET(def_tx)},
	{"macTxError", PRUETH_STAT_OFFSET(mac_txerror)},
	{"rxOverSizedFrames", PRUETH_STAT_OFFSET(rx_oversized_frames)},
	{"rxUnderSizedFrames", PRUETH_STAT_OFFSET(rx_undersized_frames)},
	{"rxCRCFrames", PRUETH_STAT_OFFSET(rx_crc_frames)},
	{"droppedPackets", PRUETH_STAT_OFFSET(dropped_packets)},

	{"tx64byte", PRUETH_STAT_OFFSET(tx64byte)},
	{"tx65_127byte", PRUETH_STAT_OFFSET(tx65_127byte)},
	{"tx128_255byte", PRUETH_STAT_OFFSET(tx128_255byte)},
	{"tx256_511byte", PRUETH_STAT_OFFSET(tx256_511byte)},
	{"tx512_1023byte", PRUETH_STAT_OFFSET(tx512_1023byte)},
	{"tx1024byte", PRUETH_STAT_OFFSET(tx1024byte)},
	{"rx64byte", PRUETH_STAT_OFFSET(rx64byte)},
	{"rx65_127byte", PRUETH_STAT_OFFSET(rx65_127byte)},
	{"rx128_255byte", PRUETH_STAT_OFFSET(rx128_255byte)},
	{"rx256_511byte", PRUETH_STAT_OFFSET(rx256_511byte)},
	{"rx512_1023byte", PRUETH_STAT_OFFSET(rx512_1023byte)},
	{"rx1024byte", PRUETH_STAT_OFFSET(rx1024byte)},

	{"sqeTestError", PRUETH_STAT_OFFSET(sqe_test_error)},
};

static int emac_get_sset_count(struct net_device *ndev, int stringset)
{
	switch (stringset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(prueth_ethtool_stats);
	default:
		return -EOPNOTSUPP;
	}
}

static void emac_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(prueth_ethtool_stats); i++) {
			memcpy(p, prueth_ethtool_stats[i].string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		break;
	}
}

static void emac_get_ethtool_stats(struct net_device *ndev,
				   struct ethtool_stats *stats, u64 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct port_statistics pstats;
	u32 val;
	int i;
	void *ptr;

	emac_get_stats(emac, &pstats);

	for (i = 0; i < ARRAY_SIZE(prueth_ethtool_stats); i++) {
		ptr = &pstats;
		ptr += prueth_ethtool_stats[i].offset;
		val = *(u32 *)ptr;
		data[i] = val;
	}
}

/* Ethtool support for EMAC adapter */
static const struct ethtool_ops emac_ethtool_ops = {
	.get_drvinfo = emac_get_drvinfo,
	.get_settings = emac_get_settings,
	.set_settings = emac_set_settings,
	.get_link = ethtool_op_get_link,
	.get_ts_info = ethtool_op_get_ts_info,
	.get_sset_count = emac_get_sset_count,
	.get_strings = emac_get_strings,
	.get_ethtool_stats = emac_get_ethtool_stats,
};

/* get emac_port corresponding to eth_node name */
static int prueth_node_port(struct device_node *eth_node)
{
	if (!strcmp(eth_node->name, "ethernet-mii0"))
		return PRUETH_PORT_MII0;
	else if (!strcmp(eth_node->name, "ethernet-mii1"))
		return PRUETH_PORT_MII1;
	else
		return -EINVAL;
}

static int prueth_netdev_init(struct prueth *prueth,
			      struct device_node *eth_node, int rx_irq)
{
	enum prueth_port port;
	u32 rx_sysevent;
	struct net_device *ndev;
	struct prueth_emac *emac;
	void __iomem *sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	void __iomem *dram0 = prueth->mem[PRUETH_MEM_DRAM0].va;
	void __iomem *dram1 = prueth->mem[PRUETH_MEM_DRAM1].va;
	const u8 *mac_addr;
	int ret;

	port = prueth_node_port(eth_node);
	if (port < 0)
		return -EINVAL;

	if (of_property_read_u32(eth_node, "sysevent-rx", &rx_sysevent)) {
		dev_err(prueth->dev, "unable to get sysevent-rx\n");
		return -EINVAL;
	}

	if (rx_sysevent >= MAX_PRU_SYS_EVENTS)
		return -EINVAL;

	ndev = alloc_etherdev(sizeof(*emac));
	if (!ndev)
		return -ENOMEM;

	emac = netdev_priv(ndev);
	prueth->emac[port] = emac;
	emac->prueth = prueth;
	emac->ndev = ndev;
	emac->port_id = port;
	emac->rx_irq = rx_irq;
	emac->rx_sysevent = rx_sysevent;
	emac->msg_enable = netif_msg_init(debug_level, PRUETH_EMAC_DEBUG);
	spin_lock_init(&emac->lock);

	switch (port) {
	case PRUETH_PORT_MII0:
		emac->rx_queue_descs = sram + HOST_QUEUE_DESC_OFFSET;
		emac->tx_queue_descs = dram0 + PORT_QUEUE_DESC_OFFSET;
		break;
	case PRUETH_PORT_MII1:
		emac->rx_queue_descs = sram + HOST_QUEUE_DESC_OFFSET;
		emac->tx_queue_descs = dram1 + PORT_QUEUE_DESC_OFFSET;
		break;
	default:
		dev_err(prueth->dev, "invalid port ID\n");
		ret = -EINVAL;
		goto free;
	}

	/* get mac address from DT and set private and netdev addr */
	mac_addr = of_get_mac_address(eth_node);
	if (mac_addr)
		ether_addr_copy(ndev->dev_addr, mac_addr);
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		eth_hw_addr_random(ndev);
		dev_warn(prueth->dev, "port %d: using random MAC addr: %pM\n",
			 port, ndev->dev_addr);
	}
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	emac->phy_node = of_parse_phandle(eth_node, "phy-handle", 0);
	if (!emac->phy_node) {
		dev_err(prueth->dev, "couldn't find phy-handle\n");
		ret = -ENODEV;
		goto free;
	}

	emac->phy_if = of_get_phy_mode(eth_node);
	if (emac->phy_if < 0) {
		dev_err(prueth->dev, "could not get phy-mode property\n");
		ret = emac->phy_if;
		goto free;
	}

	/* connect PHY */
	emac->phydev = of_phy_connect(ndev, emac->phy_node,
				      &emac_adjust_link, 0, emac->phy_if);
	if (!emac->phydev) {
		dev_err(prueth->dev, "couldn't connect to phy %s\n",
			emac->phy_node->full_name);
		ret = -ENODEV;
		goto free;
	}

	emac->phydev->advertising &= ~(ADVERTISED_1000baseT_Full |
			ADVERTISED_1000baseT_Half);
	emac->phydev->supported &= ~(SUPPORTED_1000baseT_Full |
			SUPPORTED_1000baseT_Half);

	ndev->netdev_ops = &emac_netdev_ops;
	ndev->ethtool_ops = &emac_ethtool_ops;

	netif_napi_add(ndev, &emac->napi, emac_napi_poll, EMAC_POLL_WEIGHT);

	return 0;

free:
	free_netdev(ndev);
	prueth->emac[port] = NULL;

	return ret;
}

static void prueth_netdev_exit(struct prueth *prueth,
			       struct device_node *eth_node)
{
	struct prueth_emac *emac;
	enum prueth_port port;

	port = prueth_node_port(eth_node);
	if (port < 0)
		return;

	emac = prueth->emac[port];
	if (!emac)
		return;

	dev_info(prueth->dev, "freeing port %d\n", port);

	phy_disconnect(emac->phydev);

	netif_napi_del(&emac->napi);
	free_netdev(emac->ndev);
	prueth->emac[port] = NULL;
}

static const struct of_device_id prueth_dt_match[];

static int prueth_probe(struct platform_device *pdev)
{
	struct prueth *prueth;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	struct device_node *eth_node;
	struct pruss *pruss;
	int rx0_irq, rx1_irq;
	int i;
	int ret;

	if (!np)
		return -ENODEV;	/* we don't support non DT */

	match = of_match_device(prueth_dt_match, &pdev->dev);
	if (!match || !match->data) {
		dev_err(dev, "device does not have any match data\n");
		return -ENODEV;
	}

	prueth = devm_kzalloc(dev, sizeof(*prueth), GFP_KERNEL);
	if (!prueth)
		return -ENOMEM;

	platform_set_drvdata(pdev, prueth);

	prueth->dev = dev;
	prueth->data = match->data;

	pruss = pruss_get(dev);
	if (!pruss) {
		dev_err(dev, "unable to get pruss handle\n");
		return -ENODEV;
	}
	prueth->pruss = pruss;

	prueth->pru0 = pruss_rproc_get(pruss, PRUSS_PRU0);
	if (IS_ERR(prueth->pru0)) {
		ret = PTR_ERR(prueth->pru0);
		dev_err(dev, "unable to get PRU0\n");
		goto pruss_put;
	}

	prueth->pru1 = pruss_rproc_get(pruss, PRUSS_PRU1);
	if (IS_ERR(prueth->pru1)) {
		ret = PTR_ERR(prueth->pru1);
		dev_err(dev, "unable to get PRU1\n");
		goto put_pru0;
	}

	/* Configure PRUSS */
	pruss_cfg_gpimode(pruss, prueth->pru0, PRUSS_GPI_MODE_MII);
	pruss_cfg_gpimode(pruss, prueth->pru1, PRUSS_GPI_MODE_MII);
	pruss_cfg_miirt_enable(pruss, true);
	pruss_cfg_xfr_enable(pruss, true);

	ret = -EINVAL;
	rx0_irq = pruss_host_to_mpu_irq(pruss, 2);	/* HOST_2 */
	if (rx0_irq < 0) {
		dev_err(dev, "unable to get rx0 irq\n");
		goto put_pru1;
	}

	rx1_irq = pruss_host_to_mpu_irq(pruss, 3);	/* HOST_3 */
	if (rx1_irq < 0) {
		dev_err(dev, "unable to get rx1 irq\n");
		goto put_pru1;
	}

	/* Get PRUSS mem resources */
	/* OCMC is system resource which we get separately */
	for (i = 0; i < ARRAY_SIZE(pruss_mem_ids); i++) {
		ret = pruss_request_mem_region(pruss, pruss_mem_ids[i], &prueth->mem[i]);
		if (ret) {
			dev_err(dev, "unable to get PRUSS resource %d: %d\n",
				i, ret);
			goto put_mem;
		}
	}

	prueth->sram_pool = of_get_named_gen_pool(np, "sram", 0);
	if (!prueth->sram_pool) {
		dev_err(dev, "unable to get SRAM pool\n");
		ret = -ENODEV;
		goto put_mem;
	}
	prueth->mem[PRUETH_MEM_OCMC].va =
			(void __iomem *)gen_pool_alloc(prueth->sram_pool,
						       SZ_64K);
	if (IS_ERR(prueth->mem[PRUETH_MEM_OCMC].va)) {
		ret = PTR_ERR(prueth->mem[PRUETH_MEM_OCMC].va);
		dev_err(dev, "unable to allocate OCMC resource\n");
		goto put_mem;
	}
	prueth->mem[PRUETH_MEM_OCMC].pa =
			gen_pool_virt_to_phys(prueth->sram_pool,
			(unsigned long)prueth->mem[PRUETH_MEM_OCMC].va);
	prueth->mem[PRUETH_MEM_OCMC].size = SZ_64K;
	dev_dbg(dev, "ocmc: pa %pa va %p size %#x\n",
		&prueth->mem[PRUETH_MEM_OCMC].pa,
		prueth->mem[PRUETH_MEM_OCMC].va,
		prueth->mem[PRUETH_MEM_OCMC].size);

	prueth_mii_init(prueth);

	/* setup netdev interface */
	eth_node = of_get_child_by_name(np, "ethernet-mii0");
	if (!eth_node) {
		dev_err(dev, "no ethernet-mii0 node\n");
		ret = -ENODEV;
		goto free_pool;
	}
	ret = prueth_netdev_init(prueth, eth_node, rx0_irq);
	if (ret) {
		dev_err(dev, "netdev init %s failed: %d\n",
			eth_node->name, ret);
		of_node_put(eth_node);
	} else {
		prueth->eth_node[PRUETH_PORT_MII0] = eth_node;
	}

	eth_node = of_get_child_by_name(np, "ethernet-mii1");
	if (!eth_node) {
		dev_err(dev, "no ethernet-mii1 node\n");
		ret = -ENODEV;
		goto netdev_exit;
	}
	ret = prueth_netdev_init(prueth, eth_node, rx1_irq);
	if (ret) {
		dev_err(dev, "netdev init %s failed: %d\n",
			eth_node->name, ret);
		of_node_put(eth_node);
	} else {
		prueth->eth_node[PRUETH_PORT_MII1] = eth_node;
	}

	ret = prueth_hostinit(prueth);
	if (ret) {
		dev_info(dev, "hostinit failed: %d\n", ret);
		goto netdev_exit;
	}

	/* register the network device */
	for (i = 0; i < PRUETH_PORT_MAX; i++) {
		enum prueth_port port;

		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		port = prueth_node_port(eth_node);
		if (port != PRUETH_PORT_MII0 && port != PRUETH_PORT_MII1)
			continue;

		ret = register_netdev(prueth->emac[port]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port %d\n",
				port);
			goto netdev_unregister;
		}

		prueth->registered_netdevs[i] = prueth->emac[port]->ndev;
	}

	dev_info(dev, "TI PRU ethernet driver initialized\n");

	return 0;

netdev_unregister:
	for (i = 0; i < PRUETH_PORT_MAX; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

netdev_exit:
	for (i = 0; i < PRUETH_PORT_MAX; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
		of_node_put(eth_node);
	}

free_pool:
	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->mem[PRUETH_MEM_OCMC].va, SZ_64K);

put_mem:
	for (i = PRUETH_MEM_DRAM0; i < PRUETH_MEM_OCMC; i++) {
		if (prueth->mem[i].va)
			pruss_release_mem_region(pruss, &prueth->mem[i]);
	}

put_pru1:
	pruss_rproc_put(pruss, prueth->pru1);
put_pru0:
	pruss_rproc_put(pruss, prueth->pru0);
pruss_put:
	pruss_put(prueth->pruss);

	return ret;
}

static int prueth_remove(struct platform_device *pdev)
{
	struct device_node *eth_node;
	struct prueth *prueth = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < PRUETH_PORT_MAX; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

	for (i = 0; i < PRUETH_PORT_MAX; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
		of_node_put(eth_node);
	}

	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->mem[PRUETH_MEM_OCMC].va, SZ_64K);

	for (i = PRUETH_MEM_DRAM0; i < PRUETH_MEM_OCMC; i++) {
		if (prueth->mem[i].va)
			pruss_release_mem_region(prueth->pruss, &prueth->mem[i]);
	}

	pruss_rproc_put(prueth->pruss, prueth->pru1);
	pruss_rproc_put(prueth->pruss, prueth->pru0);
	pruss_put(prueth->pruss);

	return 0;
}

/* match data for AM437x SoCs, applies to PRUSS1 only */
static struct prueth_match_data am437x_prueth_match_data = {
	.pru_fw_names = { "ti-pruss/am437x-pru0-prueth-fw.elf",
			  "ti-pruss/am437x-pru1-prueth-fw.elf", },
};

/* match data (common) for AM571x and AM572x SoCs, for both PRUSS instances */
static struct prueth_match_data am57xx_prueth_match_data = {
	.pru_fw_names = { "ti-pruss/am57xx-pru0-prueth-fw.elf",
			  "ti-pruss/am57xx-pru1-prueth-fw.elf", },
};

static const struct of_device_id prueth_dt_match[] = {
	{
		.compatible = "ti,am57-prueth",
		.data = &am57xx_prueth_match_data,
	},
	{
		.compatible = "ti,am4372-prueth",
		.data = &am437x_prueth_match_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, prueth_dt_match);

static struct platform_driver prueth_driver = {
	.probe = prueth_probe,
	.remove = prueth_remove,
	.driver = {
		.name = "prueth",
		.of_match_table = prueth_dt_match,
	},
};
module_platform_driver(prueth_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("PRU Ethernet Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(PRUETH_MODULE_VERSION);
