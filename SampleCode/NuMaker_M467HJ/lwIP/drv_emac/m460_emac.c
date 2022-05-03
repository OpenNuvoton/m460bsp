/**************************************************************************//**
 * @file     m460_emac.c
 * @version  V3.00
 * @brief    M460 EMAC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "m460_emac.h"
#include "m460_mii.h"
#include "arch/sys_arch.h"

#define printf(...)     do { }while(0)


synopGMACdevice *g_gmacdev;

static struct sk_buff sRxBuf[GMAC_CNT][RECEIVE_DESC_SIZE] __attribute__ ((aligned (64)));


/* Delay execution for given amount of ticks */
void plat_delay(uint32_t ticks)
{
    volatile uint32_t base_1us, delay;
    
    base_1us = ((SystemCoreClock/1000000) / 6);
    
    delay = base_1us * ticks;    
    while(delay--) {}
}

void EMAC_ModuleInit(uint32_t intf)
{
    SYS_ResetModule(EMAC0_RST);
    
    /* Enable EMAC module clock */
    CLK_EnableModuleClock(EMAC0_MODULE);
        
    /* Configure pins for EMAC module */
    SET_EMAC0_RMII_MDC_PE8();
    SET_EMAC0_RMII_MDIO_PE9();
    SET_EMAC0_RMII_TXD0_PE10();
    SET_EMAC0_RMII_TXD1_PE11();
    SET_EMAC0_RMII_TXEN_PE12();
    SET_EMAC0_RMII_REFCLK_PC8();
    SET_EMAC0_RMII_RXD0_PC7();
    SET_EMAC0_RMII_RXD1_PC6();
    SET_EMAC0_RMII_CRSDV_PA7();
    SET_EMAC0_RMII_RXERR_PA6();
    SET_EMAC0_PPS_PB6();
    
    g_gmacdev = &GMACdev[intf];
}

static int32_t M460_EMAC_open(int intf, uint8_t *macaddr)
{
    int32_t ret = 0;
    uint32_t i;
    struct sk_buff *skb;

    /* Attach the device to MAC struct This will configure all the required base addresses
      such as Mac base, configuration base, phy base address(out of 32 possible phys) */
    if(intf == 0)
    	synopGMAC_attach(g_gmacdev, (GMAC0MappedAddr + MACBASE), (GMAC0MappedAddr + DMABASE), DEFAULT_PHY_BASE);
    else
    	synopGMAC_attach(g_gmacdev, (GMAC1MappedAddr + MACBASE), (GMAC1MappedAddr + DMABASE), DEFAULT_PHY_BASE);
    
    synopGMAC_reset(g_gmacdev);
    
    g_gmacdev->Intf = intf;
    
    /* Lets read the version of ip in to device structure */
    synopGMAC_read_version(g_gmacdev);

    /* Check for Phy initialization */
    if(SystemCoreClock >= 250000000)
        synopGMAC_set_mdc_clk_div(g_gmacdev, GmiiCsrClk5);
    else if(SystemCoreClock >= 150000000)
        synopGMAC_set_mdc_clk_div(g_gmacdev, GmiiCsrClk4);
    else if(SystemCoreClock >= 100000000)
        synopGMAC_set_mdc_clk_div(g_gmacdev, GmiiCsrClk1);
    else if(SystemCoreClock >= 60000000)
        synopGMAC_set_mdc_clk_div(g_gmacdev, GmiiCsrClk0);
    else if(SystemCoreClock >= 35000000)
        synopGMAC_set_mdc_clk_div(g_gmacdev, GmiiCsrClk3);
    else 
        synopGMAC_set_mdc_clk_div(g_gmacdev, GmiiCsrClk2);
    g_gmacdev->ClockDivMdc = synopGMAC_get_mdc_clk_div(g_gmacdev);    
    if((ret = mii_check_phy_init(g_gmacdev)) < 0)
        printf("emac:: Init PHY FAIL.\n");

    /* Set up the tx and rx descriptor queue/ring */
    synopGMAC_setup_tx_desc_queue(g_gmacdev, TRANSMIT_DESC_SIZE, RINGMODE);
    synopGMAC_init_tx_desc_base(g_gmacdev);	// Program the transmit descriptor base address in to DmaTxBase addr

    synopGMAC_setup_rx_desc_queue(g_gmacdev, RECEIVE_DESC_SIZE, RINGMODE);
    synopGMAC_init_rx_desc_base(g_gmacdev); // Program the transmit descriptor base address in to DmaTxBase addr

    synopGMAC_dma_bus_mode_init(g_gmacdev, (DmaBurstLength32 | DmaDescriptorSkip0 | DmaDescriptor8Words));
    synopGMAC_dma_control_init(g_gmacdev, (DmaStoreAndForward | DmaTxSecondFrame| DmaRxThreshCtrl128));

    /* Initialize the mac interface */
    synopGMAC_mac_init(g_gmacdev);
    synopGMAC_promisc_enable(g_gmacdev);

    synopGMAC_pause_control(g_gmacdev); // This enables the pause control in Full duplex mode of operation

    /* IPC Checksum offloading is enabled for this driver. Should only be used if Full Ip checksumm offload engine is configured in the hardware */
    synopGMAC_enable_rx_chksum_offload(g_gmacdev);    // Enable the offload engine in the receive path
    synopGMAC_rx_tcpip_chksum_drop_enable(g_gmacdev); // This is default configuration, DMA drops the packets if error in encapsulated ethernet payload

    for(i=0; i<RECEIVE_DESC_SIZE; i ++) 
    {
        skb = &sRxBuf[intf][i];
        synopGMAC_set_rx_qptr(g_gmacdev, (u32)((u64)(skb->data) & 0xFFFFFFFF), sizeof(skb->data), (u32)((u64)skb & 0xFFFFFFFF));
    }

    /* Enable interrupt */
    synopGMAC_clear_interrupt(g_gmacdev);
    synopGMAC_enable_interrupt(g_gmacdev, DmaIntEnable);
    
    /* Enable DMA */
    synopGMAC_enable_dma_rx(g_gmacdev);
    synopGMAC_enable_dma_tx(g_gmacdev);

    synopGMAC_set_mac_address(intf, macaddr);

    return ret;
}

void EMAC_Open(uint32_t intf, uint8_t *macaddr)
{        
    M460_EMAC_open(0, macaddr);
    
    synopGMAC_promisc_enable(g_gmacdev);
    if(g_gmacdev->Speed == SPEED10)
        synopGMAC_set_mode(0, 2); // 1: 100Mbps, 2: 10Mbps
    else
        synopGMAC_set_mode(0, 1); // 1: 100Mbps, 2: 10Mbps
    
    NVIC_SetPriority(EMAC0_TXRX_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(EMAC0_TXRX_IRQn);
}
