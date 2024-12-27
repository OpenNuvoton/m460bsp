/**
 * @file
 *
 * lwIP Options Configuration
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#include <stdlib.h>

/*
 * Include user defined options first. Anything not defined in these files
 * will be set to standard values. Override anything you dont like!
 */

#define NO_SYS                          1
#define MEM_ALIGNMENT                   4
#define LWIP_STATS                      0
#define LWIP_SOCKET_SET_ERRNO           0
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0

#if !defined(__IPV6_SLACC_DEMO__)
/* TCP Maximum segment size. */
#define TCP_MSS                         1460

#define MEMP_NUM_NETCONN                8
#define MEM_SIZE                        1600
#define MEMP_NUM_PBUF                   32
#define PBUF_POOL_SIZE                  64
#define TCP_WND                         40960 //Max: 65535
#define TCP_SND_BUF                     8192
#define TCP_SND_QUEUELEN                (4 * TCP_SND_BUF/TCP_MSS)
#define MEMP_NUM_TCP_SEG                64

#define LWIP_DHCP                       1

#else
/* TCP Maximum segment size. */
#define TCP_MSS                         1460

#define MEMP_NUM_NETCONN                64
#define MEM_SIZE                        1600
#define MEMP_NUM_PBUF                   64
#define PBUF_POOL_SIZE                  128
#define TCP_WND                         40960 //Max: 65535
#define TCP_SND_BUF                     8192
#define TCP_SND_QUEUELEN                (4 * TCP_SND_BUF/TCP_MSS)
#define MEMP_NUM_TCP_SEG                64

#define MEMP_NUM_UDP_PCB                16
#define MEMP_NUM_TCP_PCB                32
#define MEMP_NUM_TCP_PCB_LISTEN         8

#define LWIP_IPV4                       1
#define LWIP_DHCP                       1
#define LWIP_IPV6                       1
#define LWIP_IPV6_DHCP6                 1
#define LWIP_RAND()                     ((uint32_t)rand())
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_IPV6_MLD                   0
#define LWIP_DNS                        0

#endif

#define LWIP_USING_HW_CHECKSUM          0
/* ---------- Checksum options ---------- */
#if (LWIP_USING_HW_CHECKSUM == 1)
#define CHECKSUM_GEN_IP                 0
#define CHECKSUM_GEN_UDP                0
#define CHECKSUM_GEN_TCP                0
#define CHECKSUM_GEN_ICMP               0
#define CHECKSUM_CHECK_IP               0
#define CHECKSUM_CHECK_UDP              0
#define CHECKSUM_CHECK_TCP              0
#define CHECKSUM_CHECK_ICMP             0
#endif

#endif /* __LWIPOPTS_H__ */
