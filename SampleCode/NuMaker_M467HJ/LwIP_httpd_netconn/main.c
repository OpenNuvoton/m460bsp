/*
 * FreeRTOS Kernel V10.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* A simple HTTP server demonstrates LwIP under FreeRTOS.
   The server��s IP address could configure statically to
   192.168.1.2, or assign by DHCP server. */

#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Demo application includes. */
#include "partest.h"
#include "flash.h"
#include "flop.h"
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "dynamic.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "countsem.h"
#include "GenQTest.h"
#include "QueueSet.h"
#include "recmutex.h"
#include "death.h"

/* Hardware and starter kit includes. */
#include "NuMicro.h"
#include "m460_emac.h"
#include "m460_mii.h"

#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernetif.h"
#include "httpserver-netconn.h"

/* Priorities for the demo application tasks. */
#if 0
#define mainFLASH_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1UL )
#define mainQUEUE_POLL_PRIORITY            ( tskIDLE_PRIORITY + 2UL )
#define mainSEM_TEST_PRIORITY              ( tskIDLE_PRIORITY + 1UL )
#define mainBLOCK_Q_PRIORITY               ( tskIDLE_PRIORITY + 2UL )
#define mainCHECK_TASK_PRIORITY            ( tskIDLE_PRIORITY + 3UL )
#else
#define mainFLASH_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1UL )
#define mainQUEUE_POLL_PRIORITY            ( tskIDLE_PRIORITY + 1UL )
#define mainSEM_TEST_PRIORITY              ( tskIDLE_PRIORITY + 1UL )
#define mainCHECK_TASK_PRIORITY            ( tskIDLE_PRIORITY + 3UL )
#endif

#define mainCHECK_TASK_STACK_SIZE            ( configMINIMAL_STACK_SIZE )

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY                        ( ( portTickType ) 5000 / portTICK_RATE_MS )

/* The LED used by the check timer. */
#define mainCHECK_LED                         ( 3UL )

/* A block time of zero simply means "don't block". */
#define mainDONT_BLOCK                        ( 0UL )

/* The period after which the check timer will expire, in ms, provided no errors
have been reported by any of the standard demo tasks.  ms are converted to the
equivalent in ticks using the portTICK_RATE_MS constant. */
#define mainCHECK_TIMER_PERIOD_MS            ( 3000UL / portTICK_RATE_MS )

/* The period at which the check timer will expire, in ms, if an error has been
reported in one of the standard demo tasks.  ms are converted to the equivalent
in ticks using the portTICK_RATE_MS constant. */
#define mainERROR_CHECK_TIMER_PERIOD_MS     ( 200UL / portTICK_RATE_MS )

/* Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 1 to create a simple demo.
Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 0 to create a much more
comprehensive test application.  See the comments at the top of this file, and
the documentation page on the http://www.FreeRTOS.org web site for more
information. */
#define mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY        0

#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif
/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


unsigned char my_mac_addr[6] = DEFAULT_MAC0_ADDRESS;
struct netif netif;
static void vWebTask( void *pvParameters );

int main(void)
{
    /* Configure the hardware ready to run the test. */
    prvSetupHardware();

    xTaskCreate( vWebTask, "Web", TCPIP_THREAD_STACKSIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );

    vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
    vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
    vStartGenericQueueTasks( tskIDLE_PRIORITY );
    vStartQueueSetTasks();


    printf("FreeRTOS is starting ...\n");

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
    will never be reached.  If the following line does execute, then there was
    insufficient FreeRTOS heap memory available for the idle and/or timer tasks
    to be created.  See the memory management section on the FreeRTOS web site
    for more details. */
    for( ;; );
}
/*-----------------------------------------------------------*/

static void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

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

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

static void prvSetupHardware( void )
{
    /* Unlock protected registers */
    SYS_UnlockReg();

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
    
    /* Init UART for printf */
    UART_Init();
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()).  */

#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 0 )
    {
        /* In this case the tick hook is used as part of the queue set test. */
        vQueueSetAccessQueueSetFromISR();
    }
#endif /* mainCREATE_SIMPLE_BLINKY_DEMO_ONLY */
}

static void vWebTask( void *pvParameters )
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

#if LWIP_DHCP   
    /* To enable LWIP_DHCP 1 in lwipopts.h */
    IP4_ADDR(&gw, 0, 0, 0, 0);
    IP4_ADDR(&ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&netmask, 0, 0, 0, 0);
#else    
    IP4_ADDR(&gw, 192, 168, 1, 1);
    IP4_ADDR(&ipaddr, 192, 168, 1, 2);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
#endif
    
    tcpip_init(NULL, NULL);

    netif_add(&netif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);

    netif_set_default(&netif);
    netif_set_up(&netif);

#if LWIP_DHCP
    printf("DHCP starting ...\n");
    
    if(dhcp_start(&netif) == ERR_OK)
    {
        while(dhcp_supplied_address(&netif) == 0)
        {
            vTaskDelay(10000);
            break;
        }
    }
    else
    {
        printf("DHCP fail\n");
        while(1) {}
    }
#endif

    printf("[ httpd_netconn ] \n");
    printf("IP address:      %s\n", ip4addr_ntoa(&netif.ip_addr));
    printf("Subnet mask:     %s\n", ip4addr_ntoa(&netif.netmask));
    printf("Default gateway: %s\n", ip4addr_ntoa(&netif.gw));
        
    http_server_netconn_init();

    vTaskSuspend( NULL );
}
