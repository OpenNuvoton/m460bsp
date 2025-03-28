/**************************************************************************//**
 * @file     retarget.c
 * @version  V3.00
 * @brief    Debug Port and Semihost Setting Source File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/


#include <stdio.h>
#include "NuMicro.h"

#if(defined(__ICCARM__) && (__VER__ >= 9020000))
    #include <LowLevelIOInterface.h>
#endif


#if defined (__ICCARM__)
    #pragma diag_suppress=Pm150
#endif

int kbhit(void);
int IsDebugFifoEmpty(void);
void _ttywrch(int ch);


char GetChar(void);
void SendChar_ToUART(int ch);
void SendChar(int ch);



#if defined(__ICCARM__)

# ifndef DEBUG_ENABLE_SEMIHOST
size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    size_t nChars = 0;

    /* Check for the command to flush all handles */
    if(handle == -1)
    {
        return 0;
    }

    /* Check for stdout and stderr      (only necessary if FILE descriptors are enabled.) */

    if(handle != 1 && handle != 2)
    {
        return -1;
    }

    for(/* Empty */; bufSize > 0; --bufSize)
    {
        SendChar(*buf);
        ++buf;
        ++nChars;
    }

    return nChars;
}


size_t __read(int handle, unsigned char* buf, size_t bufSize)
{
    size_t nChars = 0;
    /* Check for stdin      (only necessary if FILE descriptors are enabled) */
    if(handle != 0)
    {
        return -1;
    }

    for(; bufSize > 0; --bufSize)
    {
        unsigned char c;
        c = GetChar();
        if(c == 0)
            break;
        *buf++ = c;
        ++nChars;
    }
    return nChars;
}
# endif
#endif


#if (defined(__ARMCC_VERSION) || defined(__ICCARM__))
    int fgetc(FILE* stream);
    int fputc(int ch, FILE* stream);
    int ferror(FILE* stream);
#endif



#if (defined(__ARMCC_VERSION ) && (__ARMCC_VERSION >= 400000) &&  (__ARMCC_VERSION < 600000))
    /* Insist on keeping widthprec, to avoid X propagation by benign code in C-lib */
    #pragma import _printf_widthprec
#endif

#if (defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6040000)) || (defined(__ICCARM__) && (__VER__ >= 8000000))
struct __FILE
{
    int handle; /* Add whatever you need here */
};
#endif

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
# ifdef __MICROLIB
FILE __stdout;
FILE __stdin;

__WEAK __NO_RETURN
void __aeabi_assert(const char* expr, const char* file, int line)
{
    char str[12], * p;

    fputs("*** assertion failed: ", stderr);
    fputs(expr, stderr);
    fputs(", file ", stderr);
    fputs(file, stderr);
    fputs(", line ", stderr);

    p = str + sizeof(str);
    *--p = '\0';
    *--p = '\n';
    while(line > 0)
    {
        *--p = '0' + (line % 10);
        line /= 10;
    }
    fputs(p, stderr);

    for(;;);
}


__WEAK
void abort(void)
{
    for(;;);
}

# else
__asm("  .global __ARM_use_no_argv\n");
__asm("  .global __use_no_semihosting\n");


FILE __stdout;
FILE __stdin;
FILE __stderr;

void _sys_exit(int return_code)__attribute__((noreturn));
void _sys_exit(int return_code)
{
    (void) return_code;
    while(1);
}


# endif
#endif // defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)


#if (defined(__ARMCC_VERSION) || defined(__ICCARM__))
    __WEAK
    uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp);
#endif


#if defined(DEBUG_ENABLE_SEMIHOST)
#if (defined(__ARMCC_VERSION) || defined(__ICCARM__))
/* The static buffer is used to speed up the semihost */
static char g_buf[16];
static uint8_t g_buf_len = 0;
static volatile int32_t g_ICE_Conneced = 1;



void _sys_exit(int return_code)__attribute__((noreturn));

/**
 * @brief    This function is called by Hardfault handler.
 * @param    None
 * @returns  None
 * @details  This function is called by Hardfault handler and check if it is caused by __BKPT or not.
 *
 */

uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t *sp = NULL;
    uint32_t inst;

    /* Check the used stack */
    if(lr & 0x40)
    {
        /* Secure stack used */
        if(lr & 4)
            sp = (uint32_t *)psp;
        else
            sp = (uint32_t *)msp;

    }
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
    else
    {
        /* Non-secure stack used */
        if(lr & 4)
            sp = (uint32_t *)__TZ_get_PSP_NS();
        else
            sp = (uint32_t *)__TZ_get_MSP_NS();

    }
#endif

    /* Get the instruction caused the hardfault */
    if(sp != NULL)
        inst = M16(sp[6]);


    if(inst == 0xBEAB)
    {
        /*
            If the instruction is 0xBEAB, it means it is caused by BKPT without ICE connected.
            We still return for output/input message to UART.
        */
        g_ICE_Conneced = 0; // Set a flag for ICE offline
        sp[6] += 2; // return to next instruction
        return lr;  // Keep lr in R0
    }

    /* It is casued by hardfault (Not semihost). Just process the hard fault here. */
    /* TODO: Implement your hardfault handle code here */

    /*
    printf("  HardFault!\n\n");
    printf("r0  = 0x%x\n", sp[0]);
    printf("r1  = 0x%x\n", sp[1]);
    printf("r2  = 0x%x\n", sp[2]);
    printf("r3  = 0x%x\n", sp[3]);
    printf("r12 = 0x%x\n", sp[4]);
    printf("lr  = 0x%x\n", sp[5]);
    printf("pc  = 0x%x\n", sp[6]);
    printf("psr = 0x%x\n", sp[7]);
    */

    while(1) {}

}


static int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1)
{
    __BKPT(0xAB);


    return n32In_R0;
}

static int32_t SH_ReadC()
{
    return SH_DoCommand(0x07, NULL);
}

static int32_t SH_Write0(char *str)
{
    return SH_DoCommand(0x04, (int32_t)str);
}

static int32_t SH_ReportException()
{
    return SH_DoCommand(0x18, 0x20026);
}

#ifdef __ARMCC_VERSION
static int32_t SH_kbhit()
{
    return SH_DoCommand(0x101, NULL);
}
#endif



/**
 *
 * @brief      The function to process semihosted command
 * @param[in]  n32In_R0  : semihost register 0
 * @param[in]  n32In_R1  : semihost register 1
 * @param[out] pn32Out_R0: semihost register 0
 * @retval     0: No ICE debug
 * @retval     1: ICE debug
 *
 */

#endif


# ifdef __ICCARM__
void __exit(int return_code)
{

    /* Check if link with ICE */
    if(SH_ReportException() == 0)
    {
        /* Make sure all message is print out */
        while(IsDebugFifoEmpty() == 0);
    }
label:
    goto label;  /* endless loop */
}
# else
void _sys_exit(int return_code)
{
    (void)return_code;
    /* Check if link with ICE */
    if(SH_ReportException() == 0)
    {
        /* Make sure all message is print out */
        while(IsDebugFifoEmpty() == 0);
    }
label:
    goto label;  /* endless loop */
}
# endif

#else // defined(DEBUG_ENABLE_SEMIHOST)
__WEAK uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t *sp = NULL;
    uint32_t inst = 0, addr = 0, taddr, tdata;
    int32_t secure;
    uint32_t rm, rn, rt, imm5, imm8;

    /* It is casued by hardfault. Just process the hard fault */
    /* TODO: Implement your hardfault handle code here */


    /* Check the used stack */
    secure = (lr & 0x40ul) ? 1 : 0;
    if(secure)
    {
        /* Secure stack used */
        if(lr & 4UL)
        {
            sp = (uint32_t *)psp;
        }
        else
        {
            sp = (uint32_t *)msp;
        }

    }
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3)
    else
    {
        /* Non-secure stack used */
        if(lr & 4)
            sp = (uint32_t *)(__TZ_get_PSP_NS());
        else
            sp = (uint32_t *)(__TZ_get_MSP_NS());

    }
#endif

    /*
        r0  = sp[0]
        r1  = sp[1]
        r2  = sp[2]
        r3  = sp[3]
        r12 = sp[4]
        lr  = sp[5]
        pc  = sp[6]
        psr = sp[7]
    */


    printf("HardFault @ 0x%08x\n", sp[6]);
    /* Get the instruction caused the hardfault */
    if(sp != NULL)
    {
        addr = sp[6];
        inst = M16(addr);
    }

    printf("HardFault Analysis:\n");

    printf("Instruction code = %x\n", inst);

    if(inst == 0xBEAB)
    {
        printf("Execute BKPT without ICE connected\n");
    }
    else if((inst >> 12) == 5)
    {
        /* 0101xx Load/store (register offset) on page C2-327 of armv8m ref */
        rm = (inst >> 6) & 0x7;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;

        printf("LDR/STR rt=%x rm=%x rn=%x\n", rt, rm, rn);
        taddr = sp[rn] + sp[rm];
        tdata = sp[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);

    }
    else if((inst >> 13) == 3)
    {
        /* 011xxx    Load/store word/byte (immediate offset) on page C2-327 of armv8m ref */
        imm5 = (inst >> 6) & 0x1f;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;

        printf("LDR/STR rt=%x rn=%x imm5=%x\n", rt, rn, imm5);
        taddr = sp[rn] + imm5;
        tdata = sp[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);
    }
    else if((inst >> 12) == 8)
    {
        /* 1000xx    Load/store halfword (immediate offset) on page C2-328 */
        imm5 = (inst >> 6) & 0x1f;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;

        printf("LDRH/STRH rt=%x rn=%x imm5=%x\n", rt, rn, imm5);
        taddr = sp[rn] + imm5;
        tdata = sp[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);

    }
    else if((inst >> 12) == 9)
    {
        /* 1001xx    Load/store (SP-relative) on page C2-328 */
        imm8 = inst & 0xff;
        rt = (inst >> 8) & 0x7;

        printf("LDRH/STRH rt=%x imm8=%x\n", rt, imm8);
        taddr = sp[6] + imm8;
        tdata = sp[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);
    }
    else
    {
        printf("Unexpected instruction\n");
    }



    /* Or *sp to remove compiler warning */
    while(1U | *sp) {}

    return lr;
}
#endif /* defined(DEBUG_ENABLE_SEMIHOST) */


/**
 * @brief    Routine to send a char
 *
 * @param[in] ch  A character data writes to debug port
 *
 * @returns  Send value from UART debug port
 *
 * @details  Send a target char to UART debug port .
 */
void SendChar_ToUART(int ch)
{
    if((char)ch == '\n')
    {
        while(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}
        DEBUG_PORT->DAT = '\r';
    }

    while(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}
    DEBUG_PORT->DAT = (uint32_t)ch;
}

/**
 * @brief    Routine to send a char
 *
 * @param[in] ch A character data writes to debug port
 *
 * @returns  Send value from UART debug port or semihost
 *
 * @details  Send a target char to UART debug port or semihost.
 */
void SendChar(int ch)
{
#if defined(DEBUG_ENABLE_SEMIHOST)

    g_buf[g_buf_len++] = (char)ch;
    g_buf[g_buf_len] = '\0';
    if(g_buf_len + 1 >= sizeof(g_buf) || ch == '\n' || ch == '\0')
    {
        /* Send the char */
        if(g_ICE_Conneced)
        {

            if(SH_Write0(g_buf) != 0)
            {
                g_buf_len = 0;

                return;
            }
        }
        else
        {
# if (DEBUG_ENABLE_SEMIHOST == 2) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=2
            int i;

            for(i = 0; i < g_buf_len; i++)
                SendChar_ToUART(g_buf[i]);
            g_buf_len = 0;
# endif
        }
    }
#else
    SendChar_ToUART(ch);
#endif
}

/**
 * @brief    Routine to get a char
 *
 * @param    None
 *
 * @returns  Get value from UART debug port or semihost
 *
 * @details  Wait UART debug port or semihost to input a char.
 */
char GetChar(void)
{
#ifdef DEBUG_ENABLE_SEMIHOST
    int nRet;

# if defined (__ICCARM__)
    if(g_ICE_Conneced)
    {
        nRet = SH_ReadC();
        if(nRet != 0)
        {
            return nRet;
        }
    }
# else
    while(SH_kbhit())
    {
        {
            if((nRet = SH_ReadC()) != 0)
                return nRet;
        }
# endif

# if (DEBUG_ENABLE_SEMIHOST == 2) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=2
    /* Use debug port when ICE is not connected at semihost mode */
    while(!g_ICE_Conneced)
    {
        if((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (DEBUG_PORT->DAT);
        }
    }
# endif


    return (0);
#else

    while(1)
    {
        if((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            return ((char)DEBUG_PORT->DAT);
        }
    }

#endif
}

/**
 * @brief    Check any char input from UART
 *
 * @param    None
 *
 * @retval   0: No any char input
 * @retval   1: Have some char input
 *
 * @details  Check UART RSR RX EMPTY or not to determine if any char input from UART
 */

int kbhit(void)
{
    return !((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == UART_FIFOSTS_RXEMPTY_Msk);
}
/**
 * @brief    Check if debug message finished
 *
 * @param    None
 *
 * @retval   1: Message is finished
 * @retval   0: Message is transmitting.
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */

int IsDebugFifoEmpty(void)
{
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0U);
}

/**
 * @brief    C library retargetting
 *
 * @param[in]  ch  Write a character data
 *
 * @returns  None
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */

void _ttywrch(int ch)
{
    SendChar(ch);
    return;
}


/**
 * @brief      Write character to stream
 *
 * @param[in]  ch       Character to be written. The character is passed as its int promotion.
 * @param[in]  stream   Pointer to a FILE object that identifies the stream where the character is to be written.
 *
 * @returns    If there are no errors, the same character that has been written is returned.
 *             If an error occurs, EOF is returned and the error indicator is set (see ferror).
 *
 * @details    Writes a character to the stream and advances the position indicator.\n
 *             The character is written at the current position of the stream as indicated \n
 *             by the internal position indicator, which is then advanced one character.
 *
 * @note       The above descriptions are copied from http://www.cplusplus.com/reference/clibrary/cstdio/fputc/.
 *
 *
 */

int fputc(int ch, FILE *stream)
{
    (void)stream;
    SendChar(ch);
    return ch;
}


#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))

#if !defined(OS_USE_SEMIHOSTING)
int _write(int fd, char *ptr, int len)
{
    int i = len;

    while(i--)
    {
        if(*ptr == '\n')
        {
            while(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
            DEBUG_PORT->DAT = '\r';
        }

        while(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        DEBUG_PORT->DAT = *ptr++;

    }
    return len;
}

int _read(int fd, char *ptr, int len)
{

    while((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0);
    *ptr = DEBUG_PORT->DAT;
    return 1;


}
#endif

#else
/**
 * @brief      Get character from UART debug port or semihosting input
 *
 * @param[in]  stream   Pointer to a FILE object that identifies the stream on which the operation is to be performed.
 *
 * @returns    The character read from UART debug port or semihosting
 *
 * @details    For get message from debug port or semihosting.
 *
 */

int fgetc(FILE *stream)
{
    (void)stream;
    return ((int)GetChar());
}

/**
 * @brief      Check error indicator
 *
 * @param[in]  stream   Pointer to a FILE object that identifies the stream.
 *
 * @returns    If the error indicator associated with the stream was set, the function returns a nonzero value.
 *             Otherwise, it returns a zero value.
 *
 * @details    Checks if the error indicator associated with stream is set, returning a value different
 *             from zero if it is. This indicator is generally set by a previous operation on the stream that failed.
 *
 * @note       The above descriptions are copied from http://www.cplusplus.com/reference/clibrary/cstdio/ferror/.
 *
 */

int ferror(FILE *stream)
{
    (void)stream;
    return EOF;
}
#endif
