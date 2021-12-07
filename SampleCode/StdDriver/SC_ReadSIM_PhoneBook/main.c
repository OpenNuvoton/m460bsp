/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to read phone book information in the SIM card.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sclib.h"

#define SC_INTF         0 // Smartcard interface 0

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
/* The definition of commands used in this sample code and directory structures could
   be found in GSM 11.11 which is free for download from Internet.
   Different from the command defined in ISO 7816-4, CLS of SIM command is 0xA0,
   So the command defined below starting with 0xA0 */
/* Select File */
static const uint8_t g_au8SelectMF[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
static const uint8_t g_au8SelectDF_TELECOM[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x10};
static const uint8_t g_au8SelectEF_ADN[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x6F, 0x3A};
/* Get Response */
static uint8_t g_au8GetResp[] = {0xA0, 0xC0, 0x00, 0x00, 0x00};
/* Read Record */
static uint8_t g_au8ReadRec[] = {0xA0, 0xB2, 0x01, 0x04, 0x00};
/* Verify CHV, CHV = Card Holder Verification information */
static uint8_t g_au8VerifyCHV[] = {0xA0, 0x20, 0x00, 0x01, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t g_au8Buf[300];
static uint32_t g_u32Len;

void SC0_IRQHandler(void);
void GetPIN(void);
int UnlockSIM(uint32_t u32RetryCnt);
void ReadPhoneBook(uint32_t cnt);

void SYS_Init(void);
void UART_Init(void);


/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @return None
  */
void SC0_IRQHandler(void)
{
    // Please don't remove any of the function calls below
    if(SCLIB_CheckCDEvent(SC_INTF))
        return; // Card insert/remove event occurred, no need to check other event...

    SCLIB_CheckTimeOutEvent(SC_INTF);
    SCLIB_CheckTxRxEvent(SC_INTF);
    SCLIB_CheckErrorEvent(SC_INTF);

    return;
}

/**
  * @brief  Ask user to input PIN from console
  * @param  None
  * @return None
  * @details Valid input characters (0~9) are echo to console and store in command buffer.
  *         Backspace key can delete previous input digit, ESC key delete all input digits.
  *         Valid PIN length is between 4~8digits. If PIN length is shorter than 8
  *         digits, an Enter key can terminate the input procedure.
  */
void GetPIN(void)
{
    int i = 0;
    char c = 0;

    printf("Please input PIN number:");
    while(i < 8)
    {
        c = (char)getchar();
        if(c >= 0x30 && c <= 0x39)      // Valid input characters (0~9)
        {
            g_au8VerifyCHV[5 + i] = c;
            printf("%c", c);
            i++;
        }
        else if(c == 0x7F)    // DEL (Back space)
        {
            i--;
            printf("%c", c);
        }
        else if(c == 0x0D)     // Enter
        {
            if(i >= 4)  //Min CHV length is 4 digits
                break;
        }
        else if(c == 0x1B)    //ESC
        {
            printf("\nPlease input PIN number:");
            i = 0;  // retry
        }
        else
        {
            continue;
        }

    }

    // Fill remaining digits with 0xFF
    for(; i < 8; i++)
    {
        g_au8VerifyCHV[5 + i] = 0xFF;
    }

    printf("\n");

    return;
}

/**
  * @brief  Send verify command to verify CHV1
  * @param  Remaining retry count, valid values are between 3~1
  * @return Unlock SIM card success or not
  * @retval 0 Unlock success
  * @retval -1 Unlock failed
  */
int UnlockSIM(uint32_t u32RetryCnt)
{
    while(u32RetryCnt > 0)
    {
        GetPIN(); // Ask user input PIN

        if(SCLIB_StartTransmission(SC_INTF, g_au8VerifyCHV, 13, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Verify CHV failed\n");
            break;
        }
        if(g_au8Buf[0] == 0x90 || g_au8Buf[1] == 0x00)
        {
            printf("Pass\n");
            return 0;
        }
        else
        {
            u32RetryCnt--;
            printf("Failed, remaining retry count: %d\n", u32RetryCnt);
        }
    }

    printf("Oops, SIM card locked\n");

    return -1;
}

/**
  * @brief  Read phone book and print on console
  * @param  Phone book record number
  * @return None
  */
void ReadPhoneBook(uint32_t cnt)
{
    uint32_t i, j, k;

    /*
        EF_ADN structure looks like below:

        Byte            Description                         M/O Length
        1 to X          Alpha Identifier                    O   X bytes
        X+1             Length of BCD number/SSC contents   M   1 byte
        X+2             TON and NPI                         M   1 byte
        X+3 to X + 12   Dialling Number/SSC String          M   10 bytes
        X+13            Capability/Configuration Identifier M   1 byte
        X+14            Extension1 Record Identifier        M   1 byte
    */
    for(i = 1; i < cnt + 1; i++)
    {
        g_au8ReadRec[2] = (uint8_t)i;
        if(SCLIB_StartTransmission(SC_INTF, g_au8ReadRec, 5, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Read Record failed\n");
            break;
        }
        if(g_au8Buf[0] == 0xFF) // This is an empty entry
            continue;
        printf("\n======== %d ========", i);
        printf("\nName: ");
        for(j = 0; g_au8Buf[j] != 0xFF; j++)
        {
            printf("%c", g_au8Buf[j]);
        }
        while(g_au8Buf[j] == 0xFF)   // Skip reset of the Alpha Identifier bytes
            j++;

        printf("\nNumber: ");
        j += 2; // Skip Length of BCD and TNO/NPI
        for(k = 0; k < 10; k++)
        {
            if((g_au8Buf[j + k] & 0xf) != 0xF)
                printf("%c", (g_au8Buf[j + k] & 0xf) + 0x30);
            else
                break;

            if((g_au8Buf[j + k] >> 4) != 0xF)
                printf("%c", (g_au8Buf[j + k] >> 4) + 0x30);
            else
                break;
        }
    }

    printf("\n");

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(200000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));  
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable SC0 module clock and clock source from HIRC divide 3, 4MHz*/
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(3));

    /* Set SC0 multi-function pin */
    SET_SC0_PWR_PB2();
    SET_SC0_RST_PB3();
    SET_SC0_DAT_PB4();
    SET_SC0_CLK_PB5();
    SET_SC0_nCD_PC12();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

/*
    Each SIM card contains a file system, below is a simplified example

                                MF (Master File)
                                       |
                          -------------+--------------
                          |            |             |
               EF (Elementary File)    EF   DF (Dedicated File)
                                                     |
                                            ---------+--------
                                            |        |       |
                                            EF       DF      EF
                                                     |
                                                     EF
    Each file has an two byte ID, where the first byte indicates the type of file
    '3F': Master File
    '7F': Dedicated File
    '2F': Elementary File under the Master File
    '6F': Elementary File under a Dedicated File

*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i32Ret;
    uint32_t u32Retry = 0, u32Cnt, u32CHV1Disabled = 0;
    uint32_t u32SelectMF, u32SelectDF_TELECOM, u32SelectEF_ADN;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|    Read SIM Phone Book Sample Code    |\n");
    printf("+---------------------------------------+\n\n");

    printf("# This sample code reads phone book from SIM card.\n");
    printf("# I/O configuration:\n");
    printf("    SC0PWR (PB.2)  <--> smart card slot power pin\n");
    printf("    SC0RST (PB.3)  <--> smart card slot reset pin\n");
    printf("    SC0CLK (PB.5)  <--> smart card slot clock pin\n");
    printf("    SC0DAT (PB.4)  <--> smart card slot data pin\n");
    printf("    SC0CD  (PC.12) <--> smart card slot card detect pin\n");
    printf("\n");

    /* Open smartcard interface 0. SC_CD pin state LOW indicates card absent and SC_PWR pin low raise VCC pin to card */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC0_IRQn);

    /* Wait 'til card insert */
    while(SC_IsCardInserted(SC0) == FALSE) {}

    /* Activate slot 0 */
    i32Ret = SCLIB_Activate(SC_INTF, FALSE);
    if(i32Ret != SCLIB_SUCCESS)
    {
        printf("SIM card activate failed\n");
        goto exit;
    }
    else
    {
        printf("SIM card activate... PASS.\n\n");
    }

    u32SelectMF = (uint32_t)&g_au8SelectMF;
    // Select master file.
    if(SCLIB_StartTransmission(SC_INTF, (uint8_t *)u32SelectMF, 7, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
    {
        printf("Command Select MF failed\n");
        goto exit;
    }

    // If there is no error during transmission, check the response from card
    if(g_u32Len == 2 && g_au8Buf[0] == 0x9F)
    {
        // Everything goes fine, SIM card response 0x9F following by the response data length
        g_au8GetResp[4] = g_au8Buf[1]; // response data length
        // Issue "get response" command to get the response from SIM card
        if(SCLIB_StartTransmission(SC_INTF, g_au8GetResp, 5, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    // Response ends with 0x9000 means command success
    if(g_au8Buf[g_u32Len - 2] != 0x90 || g_au8Buf[g_u32Len - 1] != 0x00)
    {
        printf("Cannot select MF\n");
        goto exit;
    }
    /*
        Response of select MF, DF listed here:
        Byte    Description
        1~2     RFU
        3~4     Total amount of memory of the selected directory which is not
                allocated to any of the DFs or EFs under the selected directory
        5~6     File ID
        7       Type of File
        8~12    RFU
        13      Length of the following data
        14      File characteristics
        15      Number of DFs which are a direct child of the current directory
        16      Number of EFs which are a direct child of the current directory
        17      Number of CHVs, UNBLOCK CHVs and administrative codes
        18      RFU
        19      CHV1 status
                b8 0: secret code not initialized, 1: secret code initialized
                b7~b5 RFU
                b4~b1 Number of false presentations remaining, 0 means blocked
        20      UNBLOCK CHV1 status
        21      CHV2 status
        22      UNBLOCK CHV2 status
        23      RFU
        24~34   Reserved for the administrative management (optional)
    */

    // Read byte 19 listed in above table to check if SIM is locked
    if(g_au8Buf[18] & 0x80)
    {
        if((u32Retry = (g_au8Buf[18] & 0xF)) == 0)   //=> Blocked!!
        {
            printf("SIM locked, and unlock retry count exceed\n");
            goto exit;
        }
    }
    // Some SIM cards has file protect by CHV1, but CHV1 disabled.
    if(g_au8Buf[13] & 0x80)
    {
        printf("CHV1 disabled\n");
        u32CHV1Disabled = 1;
    }

    u32SelectDF_TELECOM = (uint32_t)&g_au8SelectDF_TELECOM;
    // Select Dedicated File DFTELECOM which contains service related information
    if(SCLIB_StartTransmission(SC_INTF, (uint8_t *)u32SelectDF_TELECOM, 7, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
    {
        printf("Command Select DF failed\n");
        goto exit;
    }
    // Don't care about the response of g_au8SelectDF_TELECOM command here as long as there's no error.


    /* Select Elementary File ADN, where ADN stands for "Abbreviated dialling numbers",
       this is the file used to store phone book */
    u32SelectEF_ADN = (uint32_t)&g_au8SelectEF_ADN;
    if(SCLIB_StartTransmission(SC_INTF, (uint8_t *)u32SelectEF_ADN, 7, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
    {
        printf("Command Select EF failed\n");
        goto exit;
    }

    if(g_u32Len == 2 && g_au8Buf[0] == 0x9F)     // response data length
    {
        // Everything goes fine, SIM card response 0x9F following by the response data length
        g_au8GetResp[4] = g_au8Buf[1];
        if(SCLIB_StartTransmission(SC_INTF, g_au8GetResp, 5, g_au8Buf, &g_u32Len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    /*
        Response of select EF listed here:
        Byte    Description
        1~2     RFU
        3~4     File size
        5~6     File ID
        7       Type of File
        8       RFU
        9~11    Access conditions. 0: ALW, 1: CHV1, 2: CHV2, 3: RFU, 4: ADM...
                Byte 9 b8~b4 for read, seek, b3~b1 for update
                Byte 10 b8~b4 for increase, b3~b1 is RFU
                Byte 11 b8~b4 for rehabilitate, b3~b1 for invalidate
        12      File status
        13      Length of the following data (byte 14 to the end)
        14      Structure of EF
        15      Length of a record
    */

    g_au8ReadRec[4] = g_au8Buf[14]; // Phone book record length
    u32Cnt = (uint32_t)((g_au8Buf[2] << 8) + g_au8Buf[3]) / g_au8Buf[14];   // Phone book record number

    // Read or update EF_ADN can be protected by CHV1, so check if CHV1 is enabled
    if(((g_au8Buf[8] & 0x10) == 0x10) && (u32CHV1Disabled == 0))    //Protect by CHV1 ?
    {
        if(UnlockSIM(u32Retry) < 0)
        {
            printf("Unlock SIM card failed\n");
            goto exit;
        }
    }

    ReadPhoneBook(u32Cnt);
    printf("Done\n");

exit:
    while(1) {}
}
