// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk-provider.h>
#include <linux/pci.h>
#include <linux/dmi.h>
#include "dwmac-toshiba.h"
#include "dwmac4.h"
#include "stmmac.h"
#include "stmmac_ptp.h"

struct toshiba_priv_data {
	int mdio_adhoc_addr;	/* mdio address for serdes & etc */
	unsigned long crossts_adj;
	bool is_pse;
};

/* This struct is used to associate PCI Function of MAC controller on a board,
 * discovered via DMI, with the address of PHY connected to the MAC. The
 * negative value of the address means that MAC controller is not connected
 * with PHY.
 */
struct stmmac_pci_func_data {
	unsigned int func;
	int phy_addr;
};

struct stmmac_pci_dmi_data {
	const struct stmmac_pci_func_data *func;
	size_t nfuncs;
};

struct stmmac_pci_info {
	int (*setup)(struct pci_dev *pdev, struct plat_stmmacenet_data *plat);
};

static int stmmac_pci_find_phy_addr(struct pci_dev *pdev,
				    const struct dmi_system_id *dmi_list)
{
	const struct stmmac_pci_func_data *func_data;
	const struct stmmac_pci_dmi_data *dmi_data;
	const struct dmi_system_id *dmi_id;
	int func = PCI_FUNC(pdev->devfn);
	size_t n;

	dmi_id = dmi_first_match(dmi_list);
	if (!dmi_id)
		return -ENODEV;

	dmi_data = dmi_id->driver_data;
	func_data = dmi_data->func;

	for (n = 0; n < dmi_data->nfuncs; n++, func_data++)
		if (func_data->func == func)
			return func_data->phy_addr;

	return -ENODEV;
}

static int serdes_status_poll(struct stmmac_priv *priv, int phyaddr,
			      int phyreg, u32 mask, u32 val)
{
	unsigned int retries = 10;
	int val_rd;

	do {
		val_rd = mdiobus_read(priv->mii, phyaddr, phyreg);
		if ((val_rd & mask) == (val & mask))
			return 0;
		udelay(POLL_DELAY_US);
	} while (--retries);

	return -ETIMEDOUT;
}

static int tc956x_serdes_powerup(struct net_device *ndev, void *priv_data)
{
	struct tc956x_priv_data *tc956x_priv = priv_data;
	struct stmmac_priv *priv = netdev_priv(ndev);
	int serdes_phy_addr = 0;
	u32 data = 0;

	if (!tc956x_priv->mdio_adhoc_addr)
		return 0;

	serdes_phy_addr = tc956x_priv->mdio_adhoc_addr;

	/* Set the serdes rate and the PCLK rate */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data &= ~SERDES_RATE_MASK;
	data &= ~SERDES_PCLK_MASK;

	if (priv->plat->max_speed == 2500)
		data |= SERDES_RATE_PCIE_GEN2 << SERDES_RATE_PCIE_SHIFT |
			SERDES_PCLK_37p5MHZ << SERDES_PCLK_SHIFT;
	else
		data |= SERDES_RATE_PCIE_GEN1 << SERDES_RATE_PCIE_SHIFT |
			SERDES_PCLK_70MHZ << SERDES_PCLK_SHIFT;

	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* assert clk_req */
	data = mdiobus_read(priv->mii, serdes_phy_addr, SERDES_GCR0);
	data |= SERDES_PLL_CLK;
	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* check for clk_ack assertion */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PLL_CLK,
				  SERDES_PLL_CLK);

	if (data) {
		dev_err(priv->device, "Serdes PLL clk request timeout\n");
		return data;
	}

	/* assert lane reset */
	data = mdiobus_read(priv->mii, serdes_phy_addr, SERDES_GCR0);
	data |= SERDES_RST;
	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* check for assert lane reset reflection */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_RST,
				  SERDES_RST);

	if (data) {
		dev_err(priv->device, "Serdes assert lane reset timeout\n");
		return data;
	}

	/*  move power state to P0 */
	data = mdiobus_read(priv->mii, serdes_phy_addr, SERDES_GCR0);

	data &= ~SERDES_PWR_ST_MASK;
	data |= SERDES_PWR_ST_P0 << SERDES_PWR_ST_SHIFT;

	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* Check for P0 state */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PWR_ST_MASK,
				  SERDES_PWR_ST_P0 << SERDES_PWR_ST_SHIFT);

	if (data) {
		dev_err(priv->device, "Serdes power state P0 timeout.\n");
		return data;
	}

	/* PSE only - ungate SGMII PHY Rx Clock */
	if (tc956x_priv->is_pse)
		mdiobus_modify(priv->mii, serdes_phy_addr, SERDES_GCR0,
			       0, SERDES_PHY_RX_CLK);

	return 0;
}

static void tc956x_serdes_powerdown(struct net_device *ndev, void *tc956x_data)
{
	struct tc956x_priv_data *tc956x_priv = tc956x_data;
	struct stmmac_priv *priv = netdev_priv(ndev);
	int serdes_phy_addr = 0;
	u32 data = 0;

	if (!tc956x_priv->mdio_adhoc_addr)
		return;

	serdes_phy_addr = tc956x_priv->mdio_adhoc_addr;

	/* PSE only - gate SGMII PHY Rx Clock */
	if (tc956x_priv->is_pse)
		mdiobus_modify(priv->mii, serdes_phy_addr, SERDES_GCR0,
			       SERDES_PHY_RX_CLK, 0);

	/*  move power state to P3 */
	data = mdiobus_read(priv->mii, serdes_phy_addr, SERDES_GCR0);

	data &= ~SERDES_PWR_ST_MASK;
	data |= SERDES_PWR_ST_P3 << SERDES_PWR_ST_SHIFT;

	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* Check for P3 state */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PWR_ST_MASK,
				  SERDES_PWR_ST_P3 << SERDES_PWR_ST_SHIFT);

	if (data) {
		dev_err(priv->device, "Serdes power state P3 timeout\n");
		return;
	}

	/* de-assert clk_req */
	data = mdiobus_read(priv->mii, serdes_phy_addr, SERDES_GCR0);
	data &= ~SERDES_PLL_CLK;
	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* check for clk_ack de-assert */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PLL_CLK,
				  (u32)~SERDES_PLL_CLK);

	if (data) {
		dev_err(priv->device, "Serdes PLL clk de-assert timeout\n");
		return;
	}

	/* de-assert lane reset */
	data = mdiobus_read(priv->mii, serdes_phy_addr, SERDES_GCR0);
	data &= ~SERDES_RST;
	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* check for de-assert lane reset reflection */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_RST,
				  (u32)~SERDES_RST);

	if (data) {
		dev_err(priv->device, "Serdes de-assert lane reset timeout\n");
		return;
	}
}

static void tc956x_speed_mode_2500(struct net_device *ndev, void *tc956x_data)
{
	struct tc956x_priv_data *tc956x_priv = tc956x_data;
	struct stmmac_priv *priv = netdev_priv(ndev);
	int serdes_phy_addr = 0;
	u32 data = 0;

	serdes_phy_addr = tc956x_priv->mdio_adhoc_addr;

	/* Determine the link speed mode: 2.5Gbps/1Gbps */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR);

	if (((data & SERDES_LINK_MODE_MASK) >> SERDES_LINK_MODE_SHIFT) ==
	    SERDES_LINK_MODE_2G5) {
		dev_info(priv->device, "Link Speed Mode: 2.5Gbps\n");
		priv->plat->max_speed = 2500;
		priv->plat->phy_interface = PHY_INTERFACE_MODE_2500BASEX;
		priv->plat->mdio_bus_data->default_an_inband = false;
	} else {
		priv->plat->max_speed = 1000;
	}
}

/* Program PTP Clock Frequency for different variant of
 * Tc956x mGBE that has slightly different GPO mapping
 */
static void tc956x_mgbe_ptp_clk_freq_config(struct stmmac_priv *priv)
{
	struct tc956x_priv_data *tc956x_priv;
	u32 gpio_value;

	tc956x_priv = (struct tc956x_priv_data *)priv->plat->bsp_priv;

	gpio_value = readl(priv->ioaddr + GMAC_GPIO_STATUS);

	if (tc956x_priv->is_pse) {
		/* For PSE GbE, use 200MHz */
		gpio_value &= ~PSE_PTP_CLK_FREQ_MASK;
		gpio_value |= PSE_PTP_CLK_FREQ_200MHZ;
	} else {
		/* For PCH GbE, use 200MHz */
		gpio_value &= ~PCH_PTP_CLK_FREQ_MASK;
		gpio_value |= PCH_PTP_CLK_FREQ_200MHZ;
	}

	writel(gpio_value, priv->ioaddr + GMAC_GPIO_STATUS);
}

static void get_arttime(struct mii_bus *mii, int tc956x_adhoc_addr,
			u64 *art_time)
{
	u64 ns;

	ns = mdiobus_read(mii, tc956x_adhoc_addr, PMC_ART_VALUE3);
	ns <<= GMAC4_ART_TIME_SHIFT;
	ns |= mdiobus_read(mii, tc956x_adhoc_addr, PMC_ART_VALUE2);
	ns <<= GMAC4_ART_TIME_SHIFT;
	ns |= mdiobus_read(mii, tc956x_adhoc_addr, PMC_ART_VALUE1);
	ns <<= GMAC4_ART_TIME_SHIFT;
	ns |= mdiobus_read(mii, tc956x_adhoc_addr, PMC_ART_VALUE0);

	*art_time = ns;
}

static int stmmac_cross_ts_isr(struct stmmac_priv *priv)
{
	return (readl(priv->ioaddr + GMAC_INT_STATUS) & GMAC_INT_TSIE);
}

static int tc956x_crosststamp(ktime_t *device,
			     struct system_counterval_t *system,
			     void *ctx)
{
	struct tc956x_priv_data *tc956x_priv;

	struct stmmac_priv *priv = (struct stmmac_priv *)ctx;
	void __iomem *ptpaddr = priv->ptpaddr;
	void __iomem *ioaddr = priv->hw->pcsr;
	unsigned long flags;
	u64 art_time = 0;
	u64 ptp_time = 0;
	u32 num_snapshot;
	u32 gpio_value;
	u32 acr_value;
	int i;

	if (!boot_cpu_has(X86_FEATURE_ART))
		return -EOPNOTSUPP;

	tc956x_priv = priv->plat->bsp_priv;

	/* Both internal crosstimestamping and external triggered event
	 * timestamping cannot be run concurrently.
	 */
	if (priv->plat->flags & STMMAC_FLAG_EXT_SNAPSHOT_EN)
		return -EBUSY;

	priv->plat->flags |= STMMAC_FLAG_INT_SNAPSHOT_EN;

	mutex_lock(&priv->aux_ts_lock);
	/* Enable Internal snapshot trigger */
	acr_value = readl(ptpaddr + PTP_ACR);
	acr_value &= ~PTP_ACR_MASK;
	switch (priv->plat->int_snapshot_num) {
	case AUX_SNAPSHOT0:
		acr_value |= PTP_ACR_ATSEN0;
		break;
	case AUX_SNAPSHOT1:
		acr_value |= PTP_ACR_ATSEN1;
		break;
	case AUX_SNAPSHOT2:
		acr_value |= PTP_ACR_ATSEN2;
		break;
	case AUX_SNAPSHOT3:
		acr_value |= PTP_ACR_ATSEN3;
		break;
	default:
		mutex_unlock(&priv->aux_ts_lock);
		priv->plat->flags &= ~STMMAC_FLAG_INT_SNAPSHOT_EN;
		return -EINVAL;
	}
	writel(acr_value, ptpaddr + PTP_ACR);

	/* Clear FIFO */
	acr_value = readl(ptpaddr + PTP_ACR);
	acr_value |= PTP_ACR_ATSFC;
	writel(acr_value, ptpaddr + PTP_ACR);
	/* Release the mutex */
	mutex_unlock(&priv->aux_ts_lock);

	/* Trigger Internal snapshot signal
	 * Create a rising edge by just toggle the GPO1 to low
	 * and back to high.
	 */
	gpio_value = readl(ioaddr + GMAC_GPIO_STATUS);
	gpio_value &= ~GMAC_GPO1;
	writel(gpio_value, ioaddr + GMAC_GPIO_STATUS);
	gpio_value |= GMAC_GPO1;
	writel(gpio_value, ioaddr + GMAC_GPIO_STATUS);

	/* Time sync done Indication - Interrupt method */
	if (!wait_event_interruptible_timeout(priv->tstamp_busy_wait,
					      stmmac_cross_ts_isr(priv),
					      HZ / 100)) {
		priv->plat->flags &= ~STMMAC_FLAG_INT_SNAPSHOT_EN;
		return -ETIMEDOUT;
	}

	num_snapshot = (readl(ioaddr + GMAC_TIMESTAMP_STATUS) &
			GMAC_TIMESTAMP_ATSNS_MASK) >>
			GMAC_TIMESTAMP_ATSNS_SHIFT;

	/* Repeat until the timestamps are from the FIFO last segment */
	for (i = 0; i < num_snapshot; i++) {
		read_lock_irqsave(&priv->ptp_lock, flags);
		stmmac_get_ptptime(priv, ptpaddr, &ptp_time);
		*device = ns_to_ktime(ptp_time);
		read_unlock_irqrestore(&priv->ptp_lock, flags);
		get_arttime(priv->mii, tc956x_priv->mdio_adhoc_addr, &art_time);
		system->cycles = art_time;
	}

	system->cycles *= tc956x_priv->crossts_adj;
	system->cs_id = CSID_X86_ART;
	priv->plat->flags &= ~STMMAC_FLAG_INT_SNAPSHOT_EN;

	return 0;
}

static void tc956x_mgbe_pse_crossts_adj(struct tc956x_priv_data *tc956x_priv,
				       int base)
{
	if (boot_cpu_has(X86_FEATURE_ART)) {
		unsigned int art_freq;

		/* On systems that support ART, ART frequency can be obtained
		 * from ECX register of CPUID leaf (0x15).
		 */
		art_freq = cpuid_ecx(ART_CPUID_LEAF);
		do_div(art_freq, base);
		tc956x_priv->crossts_adj = art_freq;
	}
}

static void common_default_data(struct plat_stmmacenet_data *plat)
{
	plat->clk_csr = 2;	/* clk_csr_i = 20-35MHz & MDC = clk_csr_i/16 */
	plat->has_gmac = 1;
	plat->force_sf_dma_mode = 1;

	plat->mdio_bus_data->needs_reset = true;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	/* Set the maxmtu to a default of JUMBO_LEN */
	plat->maxmtu = JUMBO_LEN;

	/* Set default number of RX and TX queues to use */
	plat->tx_queues_to_use = 1;
	plat->rx_queues_to_use = 1;

	/* Disable Priority config by default */
	plat->tx_queues_cfg[0].use_prio = false;
	plat->rx_queues_cfg[0].use_prio = false;

	/* Disable RX queues routing by default */
	plat->rx_queues_cfg[0].pkt_route = 0x0;
}

static struct phylink_pcs *tc956x_mgbe_select_pcs(struct stmmac_priv *priv,
						 phy_interface_t interface)
{
	/* plat->mdio_bus_data->has_xpcs has been set true, so there
	 * should always be an XPCS. The original code would always
	 * return this if present.
	 */
	return xpcs_to_phylink_pcs(priv->hw->xpcs);
}

static int tc956x_mgbe_common_data(struct pci_dev *pdev,
				  struct plat_stmmacenet_data *plat)
{
	struct fwnode_handle *fwnode;
	char clk_name[20];
	int ret;
	int i;

	plat->pdev = pdev;
	plat->phy_addr = -1;
	plat->clk_csr = 5;
	plat->has_gmac = 0;
	plat->has_gmac4 = 1;
	plat->force_sf_dma_mode = 0;
	plat->flags |= (STMMAC_FLAG_TSO_EN | STMMAC_FLAG_SPH_DISABLE);

	/* Multiplying factor to the clk_eee_i clock time
	 * period to make it closer to 100 ns. This value
	 * should be programmed such that the clk_eee_time_period *
	 * (MULT_FACT_100NS + 1) should be within 80 ns to 120 ns
	 * clk_eee frequency is 19.2Mhz
	 * clk_eee_time_period is 52ns
	 * 52ns * (1 + 1) = 104ns
	 * MULT_FACT_100NS = 1
	 */
	plat->mult_fact_100ns = 1;

	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;

	for (i = 0; i < plat->rx_queues_to_use; i++) {
		plat->rx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;
		plat->rx_queues_cfg[i].chan = i;

		/* Disable Priority config by default */
		plat->rx_queues_cfg[i].use_prio = false;

		/* Disable RX queues routing by default */
		plat->rx_queues_cfg[i].pkt_route = 0x0;
	}

	for (i = 0; i < plat->tx_queues_to_use; i++) {
		plat->tx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

		/* Disable Priority config by default */
		plat->tx_queues_cfg[i].use_prio = false;
		/* Default TX Q0 to use TSO and rest TXQ for TBS */
		if (i > 0)
			plat->tx_queues_cfg[i].tbs_en = 1;
	}

	/* FIFO size is 4096 bytes for 1 tx/rx queue */
	plat->tx_fifo_size = plat->tx_queues_to_use * 4096;
	plat->rx_fifo_size = plat->rx_queues_to_use * 4096;

	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;
	plat->tx_queues_cfg[0].weight = 0x09;
	plat->tx_queues_cfg[1].weight = 0x0A;
	plat->tx_queues_cfg[2].weight = 0x0B;
	plat->tx_queues_cfg[3].weight = 0x0C;
	plat->tx_queues_cfg[4].weight = 0x0D;
	plat->tx_queues_cfg[5].weight = 0x0E;
	plat->tx_queues_cfg[6].weight = 0x0F;
	plat->tx_queues_cfg[7].weight = 0x10;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;
	plat->dma_cfg->fixed_burst = 0;
	plat->dma_cfg->mixed_burst = 0;
	plat->dma_cfg->aal = 0;
	plat->dma_cfg->dche = true;

	plat->axi = devm_kzalloc(&pdev->dev, sizeof(*plat->axi),
				 GFP_KERNEL);
	if (!plat->axi)
		return -ENOMEM;

	plat->axi->axi_lpi_en = 0;
	plat->axi->axi_xit_frm = 0;
	plat->axi->axi_wr_osr_lmt = 1;
	plat->axi->axi_rd_osr_lmt = 1;
	plat->axi->axi_blen[0] = 4;
	plat->axi->axi_blen[1] = 8;
	plat->axi->axi_blen[2] = 16;

	plat->ptp_max_adj = plat->clk_ptp_rate;
	plat->eee_usecs_rate = plat->clk_ptp_rate;

	/* Set system clock */
	sprintf(clk_name, "%s-%s", "stmmac", pci_name(pdev));

	plat->stmmac_clk = clk_register_fixed_rate(&pdev->dev,
						   clk_name, NULL, 0,
						   plat->clk_ptp_rate);

	if (IS_ERR(plat->stmmac_clk)) {
		dev_warn(&pdev->dev, "Fail to register stmmac-clk\n");
		plat->stmmac_clk = NULL;
	}

	ret = clk_prepare_enable(plat->stmmac_clk);
	if (ret) {
		clk_unregister_fixed_rate(plat->stmmac_clk);
		return ret;
	}

	plat->ptp_clk_freq_config = tc956x_mgbe_ptp_clk_freq_config;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	/* Set the maxmtu to a default of JUMBO_LEN */
	plat->maxmtu = JUMBO_LEN;

	plat->flags |= STMMAC_FLAG_VLAN_FAIL_Q_EN;

	/* Use the last Rx queue */
	plat->vlan_fail_q = plat->rx_queues_to_use - 1;

	/* For fixed-link setup, we allow phy-mode setting */
	fwnode = dev_fwnode(&pdev->dev);
	if (fwnode) {
		int phy_mode;

		/* "phy-mode" setting is optional. If it is set,
		 *  we allow either sgmii or 1000base-x for now.
		 */
		phy_mode = fwnode_get_phy_mode(fwnode);
		if (phy_mode >= 0) {
			if (phy_mode == PHY_INTERFACE_MODE_SGMII ||
			    phy_mode == PHY_INTERFACE_MODE_1000BASEX)
				plat->phy_interface = phy_mode;
			else
				dev_warn(&pdev->dev, "Invalid phy-mode\n");
		}
	}

	/* Tc956x mgbe SGMII interface uses pcs-xcps */
	if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII ||
	    plat->phy_interface == PHY_INTERFACE_MODE_1000BASEX) {
		plat->mdio_bus_data->pcs_mask = BIT(TC956X_MGBE_XPCS_ADDR);
		plat->mdio_bus_data->default_an_inband = true;
		plat->select_pcs = tc956x_mgbe_select_pcs;
	}

	/* Ensure mdio bus scan skips tc956x serdes and pcs-xpcs */
	plat->mdio_bus_data->phy_mask = 1 << TC956X_MGBE_ADHOC_ADDR;
	plat->mdio_bus_data->phy_mask |= 1 << TC956X_MGBE_XPCS_ADDR;

	plat->int_snapshot_num = AUX_SNAPSHOT1;

	plat->crosststamp = tc956x_crosststamp;
	plat->flags &= ~STMMAC_FLAG_INT_SNAPSHOT_EN;

	/* Setup MSI vector offset specific to Tc956x mGbE controller */
	plat->msi_mac_vec = 29;
	plat->msi_lpi_vec = 28;
	plat->msi_sfty_ce_vec = 27;
	plat->msi_sfty_ue_vec = 26;
	plat->msi_rx_base_vec = 0;
	plat->msi_tx_base_vec = 1;

	return 0;
}

static int ehl_common_data(struct pci_dev *pdev,
			   struct plat_stmmacenet_data *plat)
{
	plat->rx_queues_to_use = 8;
	plat->tx_queues_to_use = 8;
	plat->flags |= STMMAC_FLAG_USE_PHY_WOL;
	plat->flags |= STMMAC_FLAG_HWTSTAMP_CORRECT_LATENCY;

	plat->safety_feat_cfg->tsoee = 1;
	plat->safety_feat_cfg->mrxpee = 1;
	plat->safety_feat_cfg->mestee = 1;
	plat->safety_feat_cfg->mrxee = 1;
	plat->safety_feat_cfg->mtxee = 1;
	plat->safety_feat_cfg->epsi = 0;
	plat->safety_feat_cfg->edpp = 0;
	plat->safety_feat_cfg->prtyen = 0;
	plat->safety_feat_cfg->tmouten = 0;

	return tc956x_mgbe_common_data(pdev, plat);
}

static int ehl_pse0_common_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	struct tc956x_priv_data *tc956x_priv = plat->bsp_priv;

	tc956x_priv->is_pse = true;
	plat->bus_id = 2;
	plat->host_dma_width = 32;

	plat->clk_ptp_rate = 200000000;

	tc956x_mgbe_pse_crossts_adj(tc956x_priv, EHL_PSE_ART_MHZ);

	return ehl_common_data(pdev, plat);
}

static int ehl_pse0_sgmii1g_data(struct pci_dev *pdev,
				 struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
	plat->speed_mode_2500 = tc956x_speed_mode_2500;
	plat->serdes_powerup = tc956x_serdes_powerup;
	plat->serdes_powerdown = tc956x_serdes_powerdown;
	return ehl_pse0_common_data(pdev, plat);
}

static struct stmmac_pci_info ehl_pse0_sgmii1g_info = {
	.setup = ehl_pse0_sgmii1g_data,
};

static int ehl_pse1_common_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	struct tc956x_priv_data *tc956x_priv = plat->bsp_priv;

	tc956x_priv->is_pse = true;
	plat->bus_id = 3;
	plat->host_dma_width = 32;

	plat->clk_ptp_rate = 200000000;

	tc956x_mgbe_pse_crossts_adj(tc956x_priv, EHL_PSE_ART_MHZ);

	return ehl_common_data(pdev, plat);
}

static int tgl_common_data(struct pci_dev *pdev,
			   struct plat_stmmacenet_data *plat)
{
	plat->rx_queues_to_use = 6;
	plat->tx_queues_to_use = 4;
	plat->clk_ptp_rate = 204800000;
	plat->speed_mode_2500 = tc956x_speed_mode_2500;

	plat->safety_feat_cfg->tsoee = 1;
	plat->safety_feat_cfg->mrxpee = 0;
	plat->safety_feat_cfg->mestee = 1;
	plat->safety_feat_cfg->mrxee = 1;
	plat->safety_feat_cfg->mtxee = 1;
	plat->safety_feat_cfg->epsi = 0;
	plat->safety_feat_cfg->edpp = 0;
	plat->safety_feat_cfg->prtyen = 0;
	plat->safety_feat_cfg->tmouten = 0;

	return tc956x_mgbe_common_data(pdev, plat);
}

static int tc956x_xgmac3_phy0_data(struct pci_dev *pdev,
			       struct plat_stmmacenet_data *plat)
{
	plat->bus_id = 1;
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
	plat->serdes_powerup = tc956x_serdes_powerup;
	plat->serdes_powerdown = tc956x_serdes_powerdown;
	return tgl_common_data(pdev, plat);
}

static struct stmmac_pci_info tc956x_xgmac3_pci_info = {
	.setup = tc956x_xgmac3_phy0_data,
};

static const struct stmmac_pci_func_data tc956x_stmmac_func_data[] = {
	{
		.func = 6,
		.phy_addr = 1,
	},
	{
		.func = 7,
		.phy_addr = 1,
	},
};

static const struct stmmac_pci_dmi_data tc956x_stmmac_dmi_data = {
	.func = tc956x_stmmac_func_data,
	.nfuncs = ARRAY_SIZE(tc956x_stmmac_func_data),
};

static int stmmac_config_single_msi(struct pci_dev *pdev,
				    struct plat_stmmacenet_data *plat,
				    struct stmmac_resources *res)
{
	int ret;

	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0) {
		dev_info(&pdev->dev, "%s: Single IRQ enablement failed\n",
			 __func__);
		return ret;
	}

	res->irq = pci_irq_vector(pdev, 0);
	res->wol_irq = res->irq;
	plat->flags &= ~STMMAC_FLAG_MULTI_MSI_EN;
	dev_info(&pdev->dev, "%s: Single IRQ enablement successful\n",
		 __func__);

	return 0;
}

static int stmmac_config_multi_msi(struct pci_dev *pdev,
				   struct plat_stmmacenet_data *plat,
				   struct stmmac_resources *res)
{
	int ret;
	int i;

	if (plat->msi_rx_base_vec >= STMMAC_MSI_VEC_MAX ||
	    plat->msi_tx_base_vec >= STMMAC_MSI_VEC_MAX) {
		dev_info(&pdev->dev, "%s: Invalid RX & TX vector defined\n",
			 __func__);
		return -1;
	}

	ret = pci_alloc_irq_vectors(pdev, 2, STMMAC_MSI_VEC_MAX,
				    PCI_IRQ_MSI | PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_info(&pdev->dev, "%s: multi MSI enablement failed\n",
			 __func__);
		return ret;
	}

	/* For RX MSI */
	for (i = 0; i < plat->rx_queues_to_use; i++) {
		res->rx_irq[i] = pci_irq_vector(pdev,
						plat->msi_rx_base_vec + i * 2);
	}

	/* For TX MSI */
	for (i = 0; i < plat->tx_queues_to_use; i++) {
		res->tx_irq[i] = pci_irq_vector(pdev,
						plat->msi_tx_base_vec + i * 2);
	}

	if (plat->msi_mac_vec < STMMAC_MSI_VEC_MAX)
		res->irq = pci_irq_vector(pdev, plat->msi_mac_vec);
	if (plat->msi_wol_vec < STMMAC_MSI_VEC_MAX)
		res->wol_irq = pci_irq_vector(pdev, plat->msi_wol_vec);
	if (plat->msi_lpi_vec < STMMAC_MSI_VEC_MAX)
		res->lpi_irq = pci_irq_vector(pdev, plat->msi_lpi_vec);
	if (plat->msi_sfty_ce_vec < STMMAC_MSI_VEC_MAX)
		res->sfty_ce_irq = pci_irq_vector(pdev, plat->msi_sfty_ce_vec);
	if (plat->msi_sfty_ue_vec < STMMAC_MSI_VEC_MAX)
		res->sfty_ue_irq = pci_irq_vector(pdev, plat->msi_sfty_ue_vec);

	plat->flags |= STMMAC_FLAG_MULTI_MSI_EN;
	dev_info(&pdev->dev, "%s: multi MSI enablement successful\n", __func__);

	return 0;
}

/**
 * tc956x_eth_pci_probe
 *
 * @pdev: pci device pointer
 * @id: pointer to table of device id/id's.
 *
 * Description: This probing function gets called for all PCI devices which
 * match the ID table and are not "owned" by other driver yet. This function
 * gets passed a "struct pci_dev *" for each device whose entry in the ID table
 * matches the device. The probe functions returns zero when the driver choose
 * to take "ownership" of the device or an error code(-ve no) otherwise.
 */
static int tc956x_eth_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct stmmac_pci_info *info = (struct stmmac_pci_info *)id->driver_data;
	struct tc956x_priv_data *tc956x_priv;
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources res;
	int ret;

	tc956x_priv = devm_kzalloc(&pdev->dev, sizeof(*tc956x_priv), GFP_KERNEL);
	if (!tc956x_priv)
		return -ENOMEM;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return -ENOMEM;

	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data)
		return -ENOMEM;

	plat->dma_cfg = devm_kzalloc(&pdev->dev, sizeof(*plat->dma_cfg),
				     GFP_KERNEL);
	if (!plat->dma_cfg)
		return -ENOMEM;

	plat->safety_feat_cfg = devm_kzalloc(&pdev->dev,
					     sizeof(*plat->safety_feat_cfg),
					     GFP_KERNEL);
	if (!plat->safety_feat_cfg)
		return -ENOMEM;

	/* Enable pci device */
	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: ERROR: failed to enable device\n",
			__func__);
		return ret;
	}

	ret = pcim_iomap_regions(pdev, BIT(0), pci_name(pdev));
	if (ret)
		return ret;

	pci_set_master(pdev);

	plat->bsp_priv = tc956x_priv;
	tc956x_priv->mdio_adhoc_addr = TC956X_MGBE_ADHOC_ADDR;
	tc956x_priv->crossts_adj = 1;

	/* Initialize all MSI vectors to invalid so that it can be set
	 * according to platform data settings below.
	 * Note: MSI vector takes value from 0 upto 31 (STMMAC_MSI_VEC_MAX)
	 */
	plat->msi_mac_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_wol_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_lpi_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_sfty_ce_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_sfty_ue_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_rx_base_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_tx_base_vec = STMMAC_MSI_VEC_MAX;

	ret = info->setup(pdev, plat);
	if (ret)
		return ret;

	memset(&res, 0, sizeof(res));
	res.addr = pcim_iomap_table(pdev)[0];

	if (plat->eee_usecs_rate > 0) {
		u32 tx_lpi_usec;

		tx_lpi_usec = (plat->eee_usecs_rate / 1000000) - 1;
		writel(tx_lpi_usec, res.addr + GMAC_1US_TIC_COUNTER);
	}

	ret = stmmac_config_multi_msi(pdev, plat, &res);
	if (ret) {
		ret = stmmac_config_single_msi(pdev, plat, &res);
		if (ret) {
			dev_err(&pdev->dev, "%s: ERROR: failed to enable IRQ\n",
				__func__);
			goto err_alloc_irq;
		}
	}

	ret = stmmac_dvr_probe(&pdev->dev, plat, &res);
	if (ret) {
		goto err_alloc_irq;
	}

	return 0;

err_alloc_irq:
	clk_disable_unprepare(plat->stmmac_clk);
	clk_unregister_fixed_rate(plat->stmmac_clk);
	return ret;
}

/**
 * tc956x_eth_pci_remove
 *
 * @pdev: pci device pointer
 * Description: this function calls the main to free the net resources
 * and releases the PCI resources.
 */
static void tc956x_eth_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);

	stmmac_dvr_remove(&pdev->dev);

	clk_disable_unprepare(priv->plat->stmmac_clk);
	clk_unregister_fixed_rate(priv->plat->stmmac_clk);
}

static int __maybe_unused tc956x_eth_pci_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int ret;

	ret = stmmac_suspend(dev);
	if (ret)
		return ret;

	ret = pci_save_state(pdev);
	if (ret)
		return ret;

	pci_wake_from_d3(pdev, true);
	pci_set_power_state(pdev, PCI_D3hot);
	return 0;
}

static int __maybe_unused tc956x_eth_pci_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int ret;

	pci_restore_state(pdev);
	pci_set_power_state(pdev, PCI_D0);

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	return stmmac_resume(dev);
}

static SIMPLE_DEV_PM_OPS(tc956x_eth_pm_ops, tc956x_eth_pci_suspend,
			 tc956x_eth_pci_resume);

#define PCI_DEVICE_ID_TC956X_XMAC		0x0700 /* FIXME: Synthetic ID, no official vendor */

#define PCI_DEVICE_ID_TC956X_DEFAULT	0x0220

static const struct pci_device_id tc956x_eth_pci_id_table[] = {
	{ PCI_DEVICE_DATA(TC956X, DEFAULT, &tc956x_xgmac3_pci_info) },
	{}
};
MODULE_DEVICE_TABLE(pci, tc956x_eth_pci_id_table);

static struct pci_driver tc956x_eth_pci_driver = {
	.name = "tc956x-eth-pci",
	.id_table = tc956x_eth_pci_id_table,
	.probe = tc956x_eth_pci_probe,
	.remove = tc956x_eth_pci_remove,
	.driver         = {
		.pm     = &tc956x_eth_pm_ops,
	},
};

module_pci_driver(tc956x_eth_pci_driver);

MODULE_DESCRIPTION("Toshiba TC956X 10/100/1000/2500 Ethernet PCI driver");
MODULE_AUTHOR("Amit Kucheria <amitk@kernel.org>");
MODULE_LICENSE("GPL v2");
