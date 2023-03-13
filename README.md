# M460 Series CMSIS BSP

To experience the powerful features of M460 in few minutes, please refer to NuMaker-PFM-M460 Board Quick Start Guide. You can select the sample code of your interest to download and execute on the M460 board. You can open the project files to build them with Keil® MDK, IAR or Eclipse, and then download and trace them on the M460 board to see how it works.

## .\Document\

- CMSIS.html<br>
	Introduction of CMSIS version 5.0. CMSIS components included CMSIS-CORE, CMSIS-Driver, CMSIS-DSP, etc.

- NuMicro M460 Series CMSIS BSP Revision History.pdf<br>
	The revision history of M460 Series BSP.

- NuMicro M460 Series Driver Reference Guide.chm<br>
	The usage of drivers in M460 Series BSP.

## .\Library\

- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V5.0 definitions by ARM® Corp.

- CryptoAccelerator<br>
	Library for mbed TLS crypto.

- Device<br>
	CMSIS compliant device header file.

- NuMaker<br>
	Specific libraries for M460 NuMaker board.

- SmartcardLib<br>
	Library for accessing a smartcard.

- StdDriver<br>
	All peripheral driver header and source files.

- UsbHostLib<br>
	USB host library source code.

## .\Sample Code\

- CortexM4<br>
	Cortex®-M4 sample code.

- Crypto<br>
	Crypto sample code using MbedTLS library.

- FreeRTOS<br>
	Simple FreeRTOS™ demo code.

- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened. The hard fault handler show some information included program counter, which is the address where the processor was executing when the hard fault occur. The listing file (or map file) can show what function and instruction that was. It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample codes for In-System-Programming.

- NuMaker_M467HJ<br>
	Sample codes for NuMaker-PFM-M460 board.

- PowerManagement<br>
	Power management sample code.

- StdDriver<br>
	Demonstrate the usage of M460 series MCU peripheral driver APIs.

- Template<br>
	A template project file for M460 series.

- XOM<br>
	Demonstrate how to create XOM library and use it.

## .\ThirdParty\

- FatFs<br>
	An open source FAT/exFAT filesystem library.

- FreeRTOS<br>
	FreeRTOS porting for M460.

- libjpeg<br>
	A software implements JPEG baseline, extended-sequential, and progressive compression processes maintained and published by the Independent JPEG Group (IJG).

- LibMAD<br>
	A MPEG audio decoder library that currently supports MPEG-1 and the MPEG-2 extension to lower sampling frequencies, as well as the de facto MPEG 2.5 format.

- lwIP<br>
	A widely used open source TCP/IP stack designed for embedded systems.

- mbedtls-3.1.0<br>
	mbed TLS offers an SSL library with an intuitive API and readable source code, so you can actually understand what the code does.

- paho.mqtt.embedded-c<br>
	Eclipse Paho MQTT C/C++ client for Embedded platforms.

- shine<br>
	A blazing fast MP3 encoding library implemented in fixed-point arithmetic.

# License

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.
M460 BSP files are provided under the Apache-2.0 license.

