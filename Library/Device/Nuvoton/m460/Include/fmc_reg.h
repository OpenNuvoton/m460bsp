/**************************************************************************//**
 * @file     fmc_reg.h
 * @version  V1.00
 * @brief    FMC register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __FMC_REG_H__
#define __FMC_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup FMC Flash Memory Controller (FMC)
    Memory Mapped Structure for FMC Controller
@{ */

typedef struct
{
    /**
     * @var FMC_T::ISPCTL
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable Bit (Write Protect)
     * |        |          |ISP function enable bit. Set this bit to enable ISP function.
     * |        |          |0 = ISP function Disabled.
     * |        |          |1 = ISP function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |BS        |Boot Select (Write Protect)
     * |        |          |When MBS in CONFIG0 is 1, set/clear this bit to select next booting from LDROM/APROM, respectively
     * |        |          |This bit also functions as chip booting status flag, which can be used to check where chip booted from
     * |        |          |This bit is initiated with the inversed value of CBS[1] (CONFIG0[7]) after any reset is happened except CPU reset (CPU is 1) or system reset (SYS) is happened
     * |        |          |0 = Booting from APROM when MBS (CONFIG0[5]) is 1.
     * |        |          |1 = Booting from LDROM when MBS (CONFIG0[5]) is 1.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |APUEN     |APROM Update Enable Bit (Write Protect)
     * |        |          |0 = APROM cannot be updated when the chip runs in APROM.
     * |        |          |1 = APROM can be updated when the chip runs in APROM.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |CFGUEN    |CONFIG Update Enable Bit (Write Protect)
     * |        |          |0 = CONFIG cannot be updated.
     * |        |          |1 = CONFIG can be updated.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |LDUEN     |LDROM Update Enable Bit (Write Protect)
     * |        |          |LDROM update enable bit.
     * |        |          |0 = LDROM cannot be updated.
     * |        |          |1 = LDROM can be updated.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |This bit needs to be cleared by writing 1 to it.
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(4) SPROM is erased/programmed if SPUEN is set to 0
     * |        |          |(5) SPROM is programmed at SPROM secured mode.
     * |        |          |(6) Page Erase command at LOCK mode with ICE connection
     * |        |          |(7) Erase or Program command at brown-out detected
     * |        |          |(8) Destination address is illegal, such as over an available range.
     * |        |          |(9) Invalid ISP commands
     * |        |          |(10) Vector address is mapping to SPROM region
     * |        |          |(11) KPROM is erased/programmed if KEYLOCK is set to 1
     * |        |          |(12) APROM(except for Data Flash) is erased/programmed if KEYLOCK is set to 1
     * |        |          |(13) LDROM is erased/programmed if KEYLOCK is set to 1
     * |        |          |(14) SPROM is erased/programmed if KEYLOCK is set to 1 and KEYENROM[1:0] are 1.
     * |        |          |(15) CONFIG is erased/programmed if KEYLOCK is set to 1 and KEYENROM[1:0] are 1
     * |        |          |(16) Invalid operations (except for chip erase) with ICE connection if SBLOCK is not 0x5A
     * |        |          |(17) Read any content of boot loader with ICE connection
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[16]    |BL        |Boot Loader Booting (Write Protect)
     * |        |          |This bit is initiated with the inversed value of MBS (CONFIG0[5])
     * |        |          |Any reset, except CPU reset (CPU is 1) or system reset (SYS), BL will be reloaded
     * |        |          |This bit is used to check chip boot from Boot Loader or not
     * |        |          |User should keep original value of this bit when updating FMC_ISPCTL register.
     * |        |          |0 = Booting from APROM or LDROM.
     * |        |          |1 = Booting from Boot Loader.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var FMC_T::ISPADDR
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADDR   |ISP Address
     * |        |          |The NuMicro M480 series is equipped with embedded flash
     * |        |          |ISPADDR[1:0] must be kept 00 for ISP 32-bit operation
     * |        |          |ISPADDR[2:0] must be kept 000 for ISP 64-bit operation.
     * |        |          |For CRC32 Checksum Calculation command, this field is the flash starting address for checksum calculation, 4 Kbytes alignment is necessary for CRC32 checksum calculation.
     * |        |          |For FLASH 32-bit Program, ISP address needs word alignment (4-byte)
     * |        |          |For FLASH 64-bit Program, ISP address needs double word alignment (8-byte).
     * @var FMC_T::ISPDAT
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation.
     * |        |          |Read data from this register after ISP read operation.
     * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT = 0xffff_ffff
     * |        |          |For Run CRC32 Checksum Calculation command, ISPDAT is the memory size (byte) and 4 Kbytes alignment
     * |        |          |For ISP Read CRC32 Checksum command, ISPDAT is the checksum result
     * |        |          |If ISPDAT = 0x0000_0000, it means that (1) the checksum calculation is in progress, or (2) the memory range for checksum calculation is incorrect
     * @var FMC_T::ISPCMD
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |CMD       |ISP Command
     * |        |          |ISP command table is shown below:
     * |        |          |0x00= FLASH Read.
     * |        |          |0x04= Read Unique ID.
     * |        |          |0x08= Read Flash All-One Result.
     * |        |          |0x0B= Read Company ID.
     * |        |          |0x0C= Read Device ID.
     * |        |          |0x0D= Read Checksum.
     * |        |          |0x21= FLASH 32-bit Program.
     * |        |          |0x22= FLASH Page Erase. Erase any page in two banks, except for OTP.
     * |        |          |0x23= FLASH Bank Erase. Erase all pages of APROM in BANK0 or BANK1.
     * |        |          |0x27= FLASH Multi-Word Program.
     * |        |          |0x28= Run Flash All-One Verification.
     * |        |          |0x2C= Bank Remap.
     * |        |          |0x2D= Run Checksum Calculation.
     * |        |          |0x2E= Vector Remap.
     * |        |          |0x40= FLASH 64-bit Read.
     * |        |          |0x61= FLASH 64-bit Program.
     * |        |          |The other commands are invalid.
     * @var FMC_T::ISPTRG
     * Offset: 0x10  ISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger (Write Protect)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP is progressed.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var FMC_T::DFBA
     * Offset: 0x14  Data Flash Base Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBA      |Data Flash Base Address
     * |        |          |This register indicates Data Flash start address. It is a read only register.
     * |        |          |The Data Flash is shared with APROM. the content of this register is loaded from CONFIG1
     * |        |          |This register is valid when DFEN (CONFIG0[0]) =0 .
     * @var FMC_T::ISPSTS
     * Offset: 0x40  ISP Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPBUSY   |ISP Busy Flag (Read Only)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |This bit is the mirror of ISPGO(FMC_ISPTRG[0]).
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP is progressed.
     * |[2:1]   |CBS       |Boot Selection of CONFIG (Read Only)
     * |        |          |This bit is initiated with the CBS (CONFIG0[7:6]) after any reset is happened except CPU reset (CPU is 1) or system reset (SYS) is happened.
     * |        |          |The following function is valid when MBS (FMC_ISPSTS[3])= 1.
     * |        |          |00 = LDROM with IAP mode.
     * |        |          |01 = LDROM without IAP mode.
     * |        |          |10 = APROM with IAP mode.
     * |        |          |11 = APROM without IAP mode.
     * |[3]     |MBS       |Boot From Boot Loader Selection Flag (Read Only)
     * |        |          |This bit is initiated with the MBS (CONFIG0[5]) after any reset is happened except CPU reset (CPU is 1) or system reset (SYS) is happened
     * |        |          |0 = Booting from Boot Loader.
     * |        |          |1 = Booting from LDROM/APROM.(.see CBS bit setting)
     * |[5]     |PGFF      |Flash Program with Fast Verification Flag (Read Only)
     * |        |          |This bit is set if data is mismatched at ISP programming verification
     * |        |          |This bit is clear by performing ISP flash erase or ISP read CID operation
     * |        |          |0 = Flash Program is success.
     * |        |          |1 = Flash Program is fail. Program data is different with data in the flash memory
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is the mirror of ISPFF (FMC_ISPCTL[6]), it needs to be cleared by writing 1 to FMC_ISPCTL[6] or FMC_ISPSTS[6]
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(4) SPROM is erased/programmed if SPUEN is set to 0
     * |        |          |(5) SPROM is programmed at SPROM secured mode.
     * |        |          |(6) Page Erase command at LOCK mode with ICE connection
     * |        |          |(7) Erase or Program command at brown-out detected
     * |        |          |(8) Destination address is illegal, such as over an available range.
     * |        |          |(9) Invalid ISP commands
     * |        |          |(10) Vector address is mapping to SPROM region.
     * |        |          |(11) KPROM is erased/programmed if KEYLOCK is set to 1
     * |        |          |(12) APROM(except for Data Flash) is erased/programmed if KEYLOCK is set to 1
     * |        |          |(13) LDROM is erased/programmed if KEYLOCK is set to 1
     * |        |          |(14) SPROM is erased/programmed if KEYLOCK is set to 1 and KEYENROM[1:0] are 1.
     * |        |          |(15) CONFIG is erased/programmed if KEYLOCK is set to 1 and KEYENROM[1:0] are 1.
     * |        |          |(16) Invalid operations (except for chip erase) with ICE connection if SBLOCK is not 0x5A
     * |        |          |(17) Read any content of boot loader with ICE connection
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |ALLONE    |Flash All-one Verification Flag
     * |        |          |This bit is set by hardware if all of flash bits are 1, and clear if flash bits are not all 1 after "Run Flash All-One Verification" complete; this bit also can be clear by writing 1
     * |        |          |0 = All of flash bits are 1 after "Run Flash All-One Verification" complete.
     * |        |          |1 = Flash bits are not all 1 after "Run Flash All-One Verification" complete.
     * |[23:9]  |VECMAP    |Vector Page Mapping Address (Read Only)
     * |        |          |All access to 0x0000_0000~0x0000_01FF is remapped to the flash memory address {VECMAP[14:0], 9u2019h000} ~ {VECMAP[14:0], 9u2019h1FF}
     * |[24]    |INTFLAG   |ISP Interrupt Flag
     * |        |          |0 = ISP Not Finished.
     * |        |          |1 = ISP done or ISPFF set.
     * |        |          |Note: This function needs to be enabled by FMC_ISPCTRL[24].
     * |[28]    |ISPCERR   |ISP Conflict Error
     * |        |          |This bit shows when FMC is doing ISP operation. User cannot access FMC_ISP_ADDR,FMC_ISPDAT,FMC_ISPCMD,FMC_ISPTRG. It would cause ISPFF.
     * |[30]    |FBS       |Flash Bank Selection
     * |        |          |This bit indicate which bank is selected to boot.
     * |        |          |0 = Booting from BANK0.
     * |        |          |1 = Booting from BANK1.
     * @var FMC_T::CYCCTL
     * Offset: 0x4C  Flash Access Cycle Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CYCLE     |Flash Access Cycle Control (Write Protect)
     * |        |          |0001 = CPU access with one wait cycle if cache miss; flash access cycle is 1;.
     * |        |          |The HCLK working frequency range range is<27MHz
     * |        |          |0010 = CPU access with two wait cycles if cache miss; flash access cycle is 2;.
     * |        |          | The optimized HCLK working frequency range is 27~54 MHz
     * |        |          |0011 = CPU access with three wait cycles if cache miss; flash access cycle is 3;.
     * |        |          |The optimized HCLK working frequency range is 54~81MHz
     * |        |          |0100 = CPU access with four wait cycles if cache miss; flash access cycle is 4;.
     * |        |          | The optimized HCLK working frequency range is81~108MHz
     * |        |          |0101 = CPU access with five wait cycles if cache miss; flash access cycle is 5;.
     * |        |          |The optimized HCLK working frequency range is 108~135MHz
     * |        |          |0110 = CPU access with six wait cycles if cache miss; flash access cycle is 6;.
     * |        |          | The optimized HCLK working frequency range is 135~162MHz
     * |        |          |0111 = CPU access with seven wait cycles if cache miss; flash access cycle is 7;.
     * |        |          | The optimized HCLK working frequency range is 162~192MHz
     * |        |          |1000 = CPU access with eight wait cycles if cache miss; flash access cycle is 8;.
     * |        |          |The optimized HCLK working frequency range is >192MHz
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var FMC_T::MPDAT0
     * Offset: 0x80  ISP Data0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT0   |ISP Data 0
     * |        |          |This register is the first 32-bit data for 32-bit/64-bit/multi-word programming, and it is also the mirror of FMC_ISPDAT, both registers keep the same data
     * @var FMC_T::MPDAT1
     * Offset: 0x84  ISP Data1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT1   |ISP Data 1
     * |        |          |This register is the second 32-bit data for 64-bit/multi-word programming.
     * @var FMC_T::MPDAT2
     * Offset: 0x88  ISP Data2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT2   |ISP Data 2
     * |        |          |This register is the third 32-bit data for multi-word programming.
     * @var FMC_T::MPDAT3
     * Offset: 0x8C  ISP Data3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT3   |ISP Data 3
     * |        |          |This register is the fourth 32-bit data for multi-word programming.
     * @var FMC_T::MPSTS
     * Offset: 0xC0  ISP Multi-Program Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MPBUSY    |ISP Multi-word Program Busy Flag (Read Only)
     * |        |          |Write 1 to start ISP Multi-Word program operation and this bit will be cleared to 0 by hardware automatically when ISP Multi-Word program operation is finished.
     * |        |          |This bit is the mirror of ISPGO(FMC_ISPTRG[0]).
     * |        |          |0 = ISP Multi-Word program operation is finished.
     * |        |          |1 = ISP Multi-Word program operation is progressed.
     * |[1]     |PPGO      |ISP Multi-program Status (Read Only)
     * |        |          |0 = ISP multi-word program operation is not active.
     * |        |          |1 = ISP multi-word program operation is in progress.
     * |[2]     |ISPFF     |ISP Fail Flag (Read Only)
     * |        |          |This bit is the mirror of ISPFF (FMC_ISPCTL[6]), it needs to be cleared by writing 1 to FMC_ISPCTL[6] or FMC_ISPSTS[6]
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(4) SPROM is erased/programmed if SPUEN is set to 0
     * |        |          |(5) SPROM is programmed at SPROM secured mode.
     * |        |          |(6) Page Erase command at LOCK mode with ICE connection
     * |        |          |(7) Erase or Program command at brown-out detected
     * |        |          |(8) Destination address is illegal, such as over an available range.
     * |        |          |(9) Invalid ISP commands
     * |        |          |(10) Vector address is mapping to SPROM region.
     * |[4]     |D0        |ISP DATA 0 Flag (Read Only)
     * |        |          |This bit is set when FMC_MPDAT0 is written and auto-clear to 0 when the FMC_MPDAT0 data is programmed to flash complete.
     * |        |          |0 = FMC_MPDAT0 register is empty, or program to flash complete.
     * |        |          |1 = FMC_MPDAT0 register has been written, and not program to flash complete.
     * |[5]     |D1        |ISP DATA 1 Flag (Read Only)
     * |        |          |This bit is set when FMC_MPDAT1 is written and auto-clear to 0 when the FMC_MPDAT1 data is programmed to flash complete.
     * |        |          |0 = FMC_MPDAT1 register is empty, or program to flash complete.
     * |        |          |1 = FMC_MPDAT1 register has been written, and not program to flash complete.
     * |[6]     |D2        |ISP DATA 2 Flag (Read Only)
     * |        |          |This bit is set when FMC_MPDAT2 is written and auto-clear to 0 when the FMC_MPDAT2 data is programmed to flash complete.
     * |        |          |0 = FMC_MPDAT2 register is empty, or program to flash complete.
     * |        |          |1 = FMC_MPDAT2 register has been written, and not program to flash complete.
     * |[7]     |D3        |ISP DATA 3 Flag (Read Only)
     * |        |          |This bit is set when FMC_MPDAT3 is written and auto-clear to 0 when the FMC_MPDAT3 data is programmed to flash complete.
     * |        |          |0 = FMC_MPDAT3 register is empty, or program to flash complete.
     * |        |          |1 = FMC_MPDAT3 register has been written, and not program to flash complete.
     * @var FMC_T::MPADDR
     * Offset: 0xC4  ISP Multi-Program Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |MPADDR    |ISP Multi-word Program Address
     * |        |          |MPADDR is the address of ISP multi-word program operation when ISPGO flag is 1.
     * |        |          |MPADDR will keep the final ISP address when ISP multi-word program is complete.
     * @var FMC_T::XOMR0STS
     * Offset: 0xD0  XOM Region 0 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SIZE      |XOM Region 0 Size (Page-aligned)
     * |        |          |SIZE is the page number of XOM Region 0.
     * |[31:8]  |BASE      |XOM Region 0 Base Address (Page-aligned)
     * |        |          |BASE is the base address of XOM Region 0.
     * @var FMC_T::XOMR1STS
     * Offset: 0xD4  XOM Region 1 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SIZE      |XOM Region 1 Size (Page-aligned)
     * |        |          |SIZE is the page number of XOM Region 1.
     * |[31:8]  |BASE      |XOM Region 1 Base Address (Page-aligned)
     * |        |          |BASE is the base address of XOM Region 1.
     * @var FMC_T::XOMR2STS
     * Offset: 0xD8  XOM Region 2 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SIZE      |XOM Region 2 Size (Page-aligned)
     * |        |          |SIZE is the page number of XOM Region 2.
     * |[31:8]  |BASE      |XOM Region 2 Base Address (Page-aligned)
     * |        |          |BASE is the base address of XOM Region 2.
     * @var FMC_T::XOMR3STS
     * Offset: 0xDC  XOM Region 3 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SIZE      |XOM Region 3 Size (Page-aligned)
     * |        |          |SIZE is the page number of XOM Region 3.
     * |[31:8]  |BASE      |XOM Region 3 Base Address (Page-aligned)
     * |        |          |BASE is the base address of XOM Region 3.
     * @var FMC_T::XOMSTS
     * Offset: 0xE0  XOM Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XOMR0ON   |XOM Region 0 On
     * |        |          |XOM Region 0 active status.
     * |        |          |0 = No active.
     * |        |          |1 = XOM region 0 is active.
     * |[1]     |XOMR1ON   |XOM Region 1 On
     * |        |          |XOM Region 1 active status.
     * |        |          |0 = No active.
     * |        |          |1 = XOM region 1 is active.
     * |[2]     |XOMR2ON   |XOM Region 2 On
     * |        |          |XOM Region 2 active status.
     * |        |          |0 = No active.
     * |        |          |1 = XOM region 2 is active.
     * |[3]     |XOMR3ON   |XOM Region 3 On
     * |        |          |XOM Region 3 active status.
     * |        |          |0 = No active.
     * |        |          |1 = XOM region 3 is active.
     * |[4]     |XOMPEF    |XOM Page Erase Function Fail
     * |        |          |XOM page erase function status. If XOMPEF is set to 1, user needs to erase XOM region again.
     * |        |          |0 = Success.
     * |        |          |1 = Fail.
     * @var FMC_T::APPROT
     * Offset: 0x110  APROM Protect Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |APPROENn	 |APROM Proect enable
    	 * | n=0,1..|          |This bit indicates which APROM region is protected.
    	 * | ..31	  |          |0 = APROM region n is not protected.
     * |        |          |1 = APROM region n is protected.
     * |        |          |Note: APROM protect region is 0x0 + n*(0x8000) to 0x7fff + n*(0x8000)
     */

    __IO uint32_t ISPCTL;                /*!< [0x0000] ISP Control Register                                             */
    __IO uint32_t ISPADDR;               /*!< [0x0004] ISP Address Register                                             */
    __IO uint32_t ISPDAT;                /*!< [0x0008] ISP Data Register                                                */
    __IO uint32_t ISPCMD;                /*!< [0x000c] ISP Command Register                                             */
    __IO uint32_t ISPTRG;                /*!< [0x0010] ISP Trigger Control Register                                     */
    __I  uint32_t DFBA;                  /*!< [0x0014] Data Flash Base Address                                          */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[10];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t ISPSTS;                /*!< [0x0040] ISP Status Register                                              */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE1[2];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t CYCCTL;                /*!< [0x004c] Flash Access Cycle Control Register                              */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE2[12];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t MPDAT0;                /*!< [0x0080] ISP Data0 Register                                               */
    __IO uint32_t MPDAT1;                /*!< [0x0084] ISP Data1 Register                                               */
    __IO uint32_t MPDAT2;                /*!< [0x0088] ISP Data2 Register                                               */
    __IO uint32_t MPDAT3;                /*!< [0x008c] ISP Data3 Register                                               */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE3[12];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t MPSTS;                 /*!< [0x00c0] ISP Multi-Program Status Register                                */
    __I  uint32_t MPADDR;                /*!< [0x00c4] ISP Multi-Program Address Register                               */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE4[2];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t XOMR0STS;              /*!< [0x00d0] XOM Region 0 Status Register                                     */
    __I  uint32_t XOMR1STS;              /*!< [0x00d4] XOM Region 1 Status Register                                     */
    __I  uint32_t XOMR2STS;              /*!< [0x00d8] XOM Region 2 Status Register                                     */
    __I  uint32_t XOMR3STS;              /*!< [0x00dc] XOM Region 3 Status Register                                     */
    __I  uint32_t XOMSTS;                /*!< [0x00e0] XOM Status Register                                              */
    __I  uint32_t RESERVE5[11];
    __IO uint32_t APPROT;                /*!< [0x0110] XOM Status Register  */

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position          */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask              */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position             */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                 */

#define FMC_ISPCTL_SPUEN_Pos             (2)                                               /*!< FMC_T::ISPCTL: SPUEN Position          */
#define FMC_ISPCTL_SPUEN_Msk             (0x1ul << FMC_ISPCTL_SPUEN_Pos)                   /*!< FMC_T::ISPCTL: SPUEN Mask              */

#define FMC_ISPCTL_APUEN_Pos             (3)                                               /*!< FMC_T::ISPCTL: APUEN Position          */
#define FMC_ISPCTL_APUEN_Msk             (0x1ul << FMC_ISPCTL_APUEN_Pos)                   /*!< FMC_T::ISPCTL: APUEN Mask              */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position         */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask             */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position          */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask              */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position          */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask              */

#define FMC_ISPCTL_BL_Pos                (16)                                              /*!< FMC_T::ISPCTL: BL Position             */
#define FMC_ISPCTL_BL_Msk                (0x1ul << FMC_ISPCTL_BL_Pos)                      /*!< FMC_T::ISPCTL: BL Mask                 */

#define FMC_ISPADDR_ISPADDR_Pos          (0)                                               /*!< FMC_T::ISPADDR: ISPADDR Position       */
#define FMC_ISPADDR_ISPADDR_Msk          (0xfffffffful << FMC_ISPADDR_ISPADDR_Pos)         /*!< FMC_T::ISPADDR: ISPADDR Mask           */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position         */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask             */

#define FMC_ISPCMD_CMD_Pos               (0)                                               /*!< FMC_T::ISPCMD: CMD Position            */
#define FMC_ISPCMD_CMD_Msk               (0x7ful << FMC_ISPCMD_CMD_Pos)                    /*!< FMC_T::ISPCMD: CMD Mask                */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position          */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask              */

#define FMC_DFBA_DFBA_Pos                (0)                                               /*!< FMC_T::DFBA: DFBA Position             */
#define FMC_DFBA_DFBA_Msk                (0xfffffffful << FMC_DFBA_DFBA_Pos)               /*!< FMC_T::DFBA: DFBA Mask                 */

#define FMC_ISPSTS_ISPBUSY_Pos           (0)                                               /*!< FMC_T::ISPSTS: ISPBUSY Position        */
#define FMC_ISPSTS_ISPBUSY_Msk           (0x1ul << FMC_ISPSTS_ISPBUSY_Pos)                 /*!< FMC_T::ISPSTS: ISPBUSY Mask            */

#define FMC_ISPSTS_CBS_Pos               (1)                                               /*!< FMC_T::ISPSTS: CBS Position            */
#define FMC_ISPSTS_CBS_Msk               (0x3ul << FMC_ISPSTS_CBS_Pos)                     /*!< FMC_T::ISPSTS: CBS Mask                */

#define FMC_ISPSTS_MBS_Pos               (3)                                               /*!< FMC_T::ISPSTS: MBS Position            */
#define FMC_ISPSTS_MBS_Msk               (0x1ul << FMC_ISPSTS_MBS_Pos)                     /*!< FMC_T::ISPSTS: MBS Mask                */

#define FMC_ISPSTS_PGFF_Pos              (5)                                               /*!< FMC_T::ISPSTS: PGFF Position           */
#define FMC_ISPSTS_PGFF_Msk              (0x1ul << FMC_ISPSTS_PGFF_Pos)                    /*!< FMC_T::ISPSTS: PGFF Mask               */

#define FMC_ISPSTS_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTS: ISPFF Position          */
#define FMC_ISPSTS_ISPFF_Msk             (0x1ul << FMC_ISPSTS_ISPFF_Pos)                   /*!< FMC_T::ISPSTS: ISPFF Mask              */

#define FMC_ISPSTS_ALLONE_Pos            (7)                                               /*!< FMC_T::ISPSTS: ALLONE Position         */
#define FMC_ISPSTS_ALLONE_Msk            (0x1ul << FMC_ISPSTS_ALLONE_Pos)                  /*!< FMC_T::ISPSTS: ALLONE Mask             */

#define FMC_ISPSTS_VECMAP_Pos            (9)                                               /*!< FMC_T::ISPSTS: VECMAP Position         */
#define FMC_ISPSTS_VECMAP_Msk            (0x7ffful << FMC_ISPSTS_VECMAP_Pos)               /*!< FMC_T::ISPSTS: VECMAP Mask             */

#define FMC_ISPSTS_INTFLAG_Pos           (24)                                              /*!< FMC_T::ISPSTS: INTFLAG Position         */
#define FMC_ISPSTS_INTFLAG_Msk           (0x01ul << FMC_ISPSTS_INTFLAG_Pos)                /*!< FMC_T::ISPSTS: INTFLAG Position         */

#define FMC_ISPSTS_ISPCERR_Pos           (28)                                              /*!< FMC_T::ISPSTS: ISPCERR Position         */
#define FMC_ISPSTS_ISPCERR_Msk           (0x01ul << FMC_ISPSTS_ISPCERR_Pos)                /*!< FMC_T::ISPSTS: ISPCERR Position         */

#define FMC_ISPSTS_FBS_Pos               (30)                                              /*!< FMC_T::ISPSTS: FBS Position           */
#define FMC_ISPSTS_FBS_Msk               (0x1ul << FMC_ISPSTS_FBS_Pos)                     /*!< FMC_T::ISPSTS: FBS Msk                */

#define FMC_CYCCTL_CYCLE_Pos             (0)                                               /*!< FMC_T::CYCCTL: CYCLE Position          */
#define FMC_CYCCTL_CYCLE_Msk             (0xful << FMC_CYCCTL_CYCLE_Pos)                   /*!< FMC_T::CYCCTL: CYCLE Mask              */

#define FMC_MPDAT0_ISPDAT0_Pos           (0)                                               /*!< FMC_T::MPDAT0: ISPDAT0 Position        */
#define FMC_MPDAT0_ISPDAT0_Msk           (0xfffffffful << FMC_MPDAT0_ISPDAT0_Pos)          /*!< FMC_T::MPDAT0: ISPDAT0 Mask            */

#define FMC_MPDAT1_ISPDAT1_Pos           (0)                                               /*!< FMC_T::MPDAT1: ISPDAT1 Position        */
#define FMC_MPDAT1_ISPDAT1_Msk           (0xfffffffful << FMC_MPDAT1_ISPDAT1_Pos)          /*!< FMC_T::MPDAT1: ISPDAT1 Mask            */

#define FMC_MPDAT2_ISPDAT2_Pos           (0)                                               /*!< FMC_T::MPDAT2: ISPDAT2 Position        */
#define FMC_MPDAT2_ISPDAT2_Msk           (0xfffffffful << FMC_MPDAT2_ISPDAT2_Pos)          /*!< FMC_T::MPDAT2: ISPDAT2 Mask            */

#define FMC_MPDAT3_ISPDAT3_Pos           (0)                                               /*!< FMC_T::MPDAT3: ISPDAT3 Position        */
#define FMC_MPDAT3_ISPDAT3_Msk           (0xfffffffful << FMC_MPDAT3_ISPDAT3_Pos)          /*!< FMC_T::MPDAT3: ISPDAT3 Mask            */

#define FMC_MPSTS_MPBUSY_Pos             (0)                                               /*!< FMC_T::MPSTS: MPBUSY Position          */
#define FMC_MPSTS_MPBUSY_Msk             (0x1ul << FMC_MPSTS_MPBUSY_Pos)                   /*!< FMC_T::MPSTS: MPBUSY Mask              */

#define FMC_MPSTS_PPGO_Pos               (1)                                               /*!< FMC_T::MPSTS: PPGO Position            */
#define FMC_MPSTS_PPGO_Msk               (0x1ul << FMC_MPSTS_PPGO_Pos)                     /*!< FMC_T::MPSTS: PPGO Mask                */

#define FMC_MPSTS_ISPFF_Pos              (2)                                               /*!< FMC_T::MPSTS: ISPFF Position           */
#define FMC_MPSTS_ISPFF_Msk              (0x1ul << FMC_MPSTS_ISPFF_Pos)                    /*!< FMC_T::MPSTS: ISPFF Mask               */

#define FMC_MPSTS_D0_Pos                 (4)                                               /*!< FMC_T::MPSTS: D0 Position              */
#define FMC_MPSTS_D0_Msk                 (0x1ul << FMC_MPSTS_D0_Pos)                       /*!< FMC_T::MPSTS: D0 Mask                  */

#define FMC_MPSTS_D1_Pos                 (5)                                               /*!< FMC_T::MPSTS: D1 Position              */
#define FMC_MPSTS_D1_Msk                 (0x1ul << FMC_MPSTS_D1_Pos)                       /*!< FMC_T::MPSTS: D1 Mask                  */

#define FMC_MPSTS_D2_Pos                 (6)                                               /*!< FMC_T::MPSTS: D2 Position              */
#define FMC_MPSTS_D2_Msk                 (0x1ul << FMC_MPSTS_D2_Pos)                       /*!< FMC_T::MPSTS: D2 Mask                  */

#define FMC_MPSTS_D3_Pos                 (7)                                               /*!< FMC_T::MPSTS: D3 Position              */
#define FMC_MPSTS_D3_Msk                 (0x1ul << FMC_MPSTS_D3_Pos)                       /*!< FMC_T::MPSTS: D3 Mask                  */

#define FMC_MPADDR_MPADDR_Pos            (0)                                               /*!< FMC_T::MPADDR: MPADDR Position         */
#define FMC_MPADDR_MPADDR_Msk            (0xfffffffful << FMC_MPADDR_MPADDR_Pos)           /*!< FMC_T::MPADDR: MPADDR Mask             */

#define FMC_XOMR0STS_SIZE_Pos            (0)                                               /*!< FMC_T::XOMR0STS: SIZE Position         */
#define FMC_XOMR0STS_SIZE_Msk            (0xfful << FMC_XOMR0STS_SIZE_Pos)                 /*!< FMC_T::XOMR0STS: SIZE Mask             */

#define FMC_XOMR0STS_BASE_Pos            (8)                                               /*!< FMC_T::XOMR0STS: BASE Position         */
#define FMC_XOMR0STS_BASE_Msk            (0xfffffful << FMC_XOMR0STS_BASE_Pos)             /*!< FMC_T::XOMR0STS: BASE Mask             */

#define FMC_XOMR1STS_SIZE_Pos            (0)                                               /*!< FMC_T::XOMR1STS: SIZE Position         */
#define FMC_XOMR1STS_SIZE_Msk            (0xfful << FMC_XOMR1STS_SIZE_Pos)                 /*!< FMC_T::XOMR1STS: SIZE Mask             */

#define FMC_XOMR1STS_BASE_Pos            (8)                                               /*!< FMC_T::XOMR1STS: BASE Position         */
#define FMC_XOMR1STS_BASE_Msk            (0xfffffful << FMC_XOMR1STS_BASE_Pos)             /*!< FMC_T::XOMR1STS: BASE Mask             */

#define FMC_XOMR2STS_SIZE_Pos            (0)                                               /*!< FMC_T::XOMR2STS: SIZE Position         */
#define FMC_XOMR2STS_SIZE_Msk            (0xfful << FMC_XOMR2STS_SIZE_Pos)                 /*!< FMC_T::XOMR2STS: SIZE Mask             */

#define FMC_XOMR2STS_BASE_Pos            (8)                                               /*!< FMC_T::XOMR2STS: BASE Position         */
#define FMC_XOMR2STS_BASE_Msk            (0xfffffful << FMC_XOM2STS_BASE_Pos)              /*!< FMC_T::XOMR2STS: BASE Mask             */

#define FMC_XOMR3STS_SIZE_Pos            (0)                                               /*!< FMC_T::XOMR3STS: SIZE Position         */
#define FMC_XOMR3STS_SIZE_Msk            (0xfful << FMC_XOMR3STS_SIZE_Pos)                 /*!< FMC_T::XOMR3STS: SIZE Mask             */

#define FMC_XOMR3STS_BASE_Pos            (8)                                               /*!< FMC_T::XOMR3STS: BASE Position         */
#define FMC_XOMR3STS_BASE_Msk            (0xfffffful << FMC_XOMR3STS_BASE_Pos)             /*!< FMC_T::XOMR3STS: BASE Mask             */

#define FMC_XOMSTS_XOMR0ON_Pos           (0)                                               /*!< FMC_T::XOMSTS: XOMR0ON Position        */
#define FMC_XOMSTS_XOMR0ON_Msk           (0x1ul << FMC_XOMSTS_XOMR0ON_Pos)                 /*!< FMC_T::XOMSTS: XOMR0ON Mask            */

#define FMC_XOMSTS_XOMR1ON_Pos           (1)                                               /*!< FMC_T::XOMSTS: XOMR1ON Position        */
#define FMC_XOMSTS_XOMR1ON_Msk           (0x1ul << FMC_XOMSTS_XOMR1ON_Pos)                 /*!< FMC_T::XOMSTS: XOMR1ON Mask            */

#define FMC_XOMSTS_XOMR2ON_Pos           (2)                                               /*!< FMC_T::XOMSTS: XOMR2ON Position        */
#define FMC_XOMSTS_XOMR2ON_Msk           (0x1ul << FMC_XOMSTS_XOMR2ON_Pos)                 /*!< FMC_T::XOMSTS: XOMR2ON Mask            */

#define FMC_XOMSTS_XOMR3ON_Pos           (3)                                               /*!< FMC_T::XOMSTS: XOMR3ON Position        */
#define FMC_XOMSTS_XOMR3ON_Msk           (0x1ul << FMC_XOMSTS_XOMR3ON_Pos)                 /*!< FMC_T::XOMSTS: XOMR3ON Mask            */

#define FMC_XOMSTS_XOMPEF_Pos            (4)                                               /*!< FMC_T::XOMSTS: XOMPEF Position         */
#define FMC_XOMSTS_XOMPEF_Msk            (0x1ul << FMC_XOMSTS_XOMPEF_Pos)                  /*!< FMC_T::XOMSTS: XOMPEF Mask             */

#define FMC_APPROT_APPROEN0_Pos          (0)                                               /*!< FMC_T::APPROT: APPROEN0 Position       */
#define FMC_APPROT_APPROEN0_Msk          (0x1ul << FMC_APPROT_APPROEN0_Pos)                /*!< FMC_T::APPROT: APPROEN0 Mask           */

#define FMC_APPROT_APPROEN1_Pos          (1)                                               /*!< FMC_T::APPROT: APPROEN1 Position       */
#define FMC_APPROT_APPROEN1_Msk          (0x1ul << FMC_APPROT_APPROEN1_Pos)                /*!< FMC_T::APPROT: APPROEN1 Mask           */

#define FMC_APPROT_APPROEN2_Pos          (2)                                               /*!< FMC_T::APPROT: APPROEN2 Position       */
#define FMC_APPROT_APPROEN2_Msk          (0x1ul << FMC_APPROT_APPROEN2_Pos)                /*!< FMC_T::APPROT: APPROEN2 Mask           */

#define FMC_APPROT_APPROEN3_Pos          (3)                                               /*!< FMC_T::APPROT: APPROEN3 Position       */
#define FMC_APPROT_APPROEN3_Msk          (0x1ul << FMC_APPROT_APPROEN3_Pos)                /*!< FMC_T::APPROT: APPROEN3 Mask           */

#define FMC_APPROT_APPROEN4_Pos          (4)                                               /*!< FMC_T::APPROT: APPROEN4 Position       */
#define FMC_APPROT_APPROEN4_Msk          (0x1ul << FMC_APPROT_APPROEN4_Pos)                /*!< FMC_T::APPROT: APPROEN4 Mask           */

#define FMC_APPROT_APPROEN5_Pos          (5)                                               /*!< FMC_T::APPROT: APPROEN5 Position       */
#define FMC_APPROT_APPROEN5_Msk          (0x1ul << FMC_APPROT_APPROEN5_Pos)                /*!< FMC_T::APPROT: APPROEN5 Mask           */

#define FMC_APPROT_APPROEN6_Pos          (6)                                               /*!< FMC_T::APPROT: APPROEN6 Position       */
#define FMC_APPROT_APPROEN6_Msk          (0x1ul << FMC_APPROT_APPROEN6_Pos)                /*!< FMC_T::APPROT: APPROEN6 Mask           */

#define FMC_APPROT_APPROEN7_Pos          (7)                                               /*!< FMC_T::APPROT: APPROEN7 Position       */
#define FMC_APPROT_APPROEN7_Msk          (0x1ul << FMC_APPROT_APPROEN7_Pos)                /*!< FMC_T::APPROT: APPROEN7 Mask           */

#define FMC_APPROT_APPROEN8_Pos          (8)                                               /*!< FMC_T::APPROT: APPROEN8 Position       */
#define FMC_APPROT_APPROEN8_Msk          (0x1ul << FMC_APPROT_APPROEN8_Pos)                /*!< FMC_T::APPROT: APPROEN8 Mask           */

#define FMC_APPROT_APPROEN9_Pos          (9)                                               /*!< FMC_T::APPROT: APPROEN9 Position       */
#define FMC_APPROT_APPROEN9_Msk          (0x1ul << FMC_APPROT_APPROEN9_Pos)                /*!< FMC_T::APPROT: APPROEN9 Mask           */

#define FMC_APPROT_APPROEN10_Pos         (10)                                              /*!< FMC_T::APPROT: APPROEN10 Position      */
#define FMC_APPROT_APPROEN10_Msk         (0x1ul << FMC_APPROT_APPROEN10_Pos)               /*!< FMC_T::APPROT: APPROEN10 Mask          */

#define FMC_APPROT_APPROEN11_Pos         (11)                                              /*!< FMC_T::APPROT: APPROEN11 Position      */
#define FMC_APPROT_APPROEN11_Msk         (0x1ul << FMC_APPROT_APPROEN11_Pos)               /*!< FMC_T::APPROT: APPROEN11 Mask          */

#define FMC_APPROT_APPROEN12_Pos         (12)                                              /*!< FMC_T::APPROT: APPROEN12 Position      */
#define FMC_APPROT_APPROEN12_Msk         (0x1ul << FMC_APPROT_APPROEN12_Pos)               /*!< FMC_T::APPROT: APPROEN12 Mask          */

#define FMC_APPROT_APPROEN13_Pos         (13)                                              /*!< FMC_T::APPROT: APPROEN13 Position      */
#define FMC_APPROT_APPROEN13_Msk         (0x1ul << FMC_APPROT_APPROEN13_Pos)               /*!< FMC_T::APPROT: APPROEN13 Mask          */

#define FMC_APPROT_APPROEN14_Pos         (14)                                              /*!< FMC_T::APPROT: APPROEN14 Position      */
#define FMC_APPROT_APPROEN14_Msk         (0x1ul << FMC_APPROT_APPROEN14_Pos)               /*!< FMC_T::APPROT: APPROEN14 Mask          */

#define FMC_APPROT_APPROEN15_Pos         (15)                                              /*!< FMC_T::APPROT: APPROEN15 Position      */
#define FMC_APPROT_APPROEN15_Msk         (0x1ul << FMC_APPROT_APPROEN15_Pos)               /*!< FMC_T::APPROT: APPROEN15 Mask          */

#define FMC_APPROT_APPROEN16_Pos         (16)                                              /*!< FMC_T::APPROT: APPROEN16 Position      */
#define FMC_APPROT_APPROEN16_Msk         (0x1ul << FMC_APPROT_APPROEN16_Pos)               /*!< FMC_T::APPROT: APPROEN16 Mask          */

#define FMC_APPROT_APPROEN17_Pos         (17)                                              /*!< FMC_T::APPROT: APPROEN17 Position      */
#define FMC_APPROT_APPROEN17_Msk         (0x1ul << FMC_APPROT_APPROEN17_Pos)               /*!< FMC_T::APPROT: APPROEN17 Mask          */

#define FMC_APPROT_APPROEN18_Pos         (18)                                              /*!< FMC_T::APPROT: APPROEN18 Position      */
#define FMC_APPROT_APPROEN18_Msk         (0x1ul << FMC_APPROT_APPROEN18_Pos)               /*!< FMC_T::APPROT: APPROEN18 Mask          */

#define FMC_APPROT_APPROEN19_Pos         (19)                                              /*!< FMC_T::APPROT: APPROEN19 Position      */
#define FMC_APPROT_APPROEN19_Msk         (0x1ul << FMC_APPROT_APPROEN19_Pos)               /*!< FMC_T::APPROT: APPROEN19 Mask          */

#define FMC_APPROT_APPROEN20_Pos         (20)                                              /*!< FMC_T::APPROT: APPROEN20 Position      */
#define FMC_APPROT_APPROEN20_Msk         (0x1ul << FMC_APPROT_APPROEN20_Pos)               /*!< FMC_T::APPROT: APPROEN20 Mask          */

#define FMC_APPROT_APPROEN21_Pos         (21)                                              /*!< FMC_T::APPROT: APPROEN21 Position      */
#define FMC_APPROT_APPROEN21_Msk         (0x1ul << FMC_APPROT_APPROEN21_Pos)               /*!< FMC_T::APPROT: APPROEN21 Mask          */

#define FMC_APPROT_APPROEN22_Pos         (22)                                              /*!< FMC_T::APPROT: APPROEN22 Position      */
#define FMC_APPROT_APPROEN22_Msk         (0x1ul << FMC_APPROT_APPROEN22_Pos)               /*!< FMC_T::APPROT: APPROEN22 Mask          */

#define FMC_APPROT_APPROEN23_Pos         (23)                                              /*!< FMC_T::APPROT: APPROEN23 Position      */
#define FMC_APPROT_APPROEN23_Msk         (0x1ul << FMC_APPROT_APPROEN23_Pos)               /*!< FMC_T::APPROT: APPROEN23 Mask          */

#define FMC_APPROT_APPROEN24_Pos         (24)                                              /*!< FMC_T::APPROT: APPROEN24 Position      */
#define FMC_APPROT_APPROEN24_Msk         (0x1ul << FMC_APPROT_APPROEN24_Pos)               /*!< FMC_T::APPROT: APPROEN24 Mask          */

#define FMC_APPROT_APPROEN25_Pos         (25)                                              /*!< FMC_T::APPROT: APPROEN25 Position      */
#define FMC_APPROT_APPROEN25_Msk         (0x1ul << FMC_APPROT_APPROEN25_Pos)               /*!< FMC_T::APPROT: APPROEN25 Mask          */

#define FMC_APPROT_APPROEN26_Pos         (26)                                              /*!< FMC_T::APPROT: APPROEN26 Position      */
#define FMC_APPROT_APPROEN26_Msk         (0x1ul << FMC_APPROT_APPROEN26_Pos)               /*!< FMC_T::APPROT: APPROEN26 Mask          */

#define FMC_APPROT_APPROEN27_Pos         (27)                                              /*!< FMC_T::APPROT: APPROEN27 Position      */
#define FMC_APPROT_APPROEN27_Msk         (0x1ul << FMC_APPROT_APPROEN27_Pos)               /*!< FMC_T::APPROT: APPROEN27 Mask          */

#define FMC_APPROT_APPROEN28_Pos         (28)                                              /*!< FMC_T::APPROT: APPROEN28 Position      */
#define FMC_APPROT_APPROEN28_Msk         (0x1ul << FMC_APPROT_APPROEN28_Pos)               /*!< FMC_T::APPROT: APPROEN28 Mask          */

#define FMC_APPROT_APPROEN29_Pos         (29)                                              /*!< FMC_T::APPROT: APPROEN29 Position      */
#define FMC_APPROT_APPROEN29_Msk         (0x1ul << FMC_APPROT_APPROEN29_Pos)               /*!< FMC_T::APPROT: APPROEN29 Mask          */

#define FMC_APPROT_APPROEN30_Pos         (30)                                              /*!< FMC_T::APPROT: APPROEN30 Position      */
#define FMC_APPROT_APPROEN30_Msk         (0x1ul << FMC_APPROT_APPROEN30_Pos)               /*!< FMC_T::APPROT: APPROEN30 Mask          */

#define FMC_APPROT_APPROEN31_Pos         (31)                                              /*!< FMC_T::APPROT: APPROEN31 Position      */
#define FMC_APPROT_APPROEN31_Msk         (0x1ul << FMC_APPROT_APPROEN31_Pos)               /*!< FMC_T::APPROT: APPROEN31 Mask          */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __FMC_REG_H__ */
