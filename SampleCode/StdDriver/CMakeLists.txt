create_project(ACMP_CompareDAC ${CMAKE_CURRENT_LIST_DIR}/ACMP_CompareDAC/main.c)
create_project(ACMP_CompareVBG ${CMAKE_CURRENT_LIST_DIR}/ACMP_CompareVBG/main.c)
create_project(ACMP_Wakeup ${CMAKE_CURRENT_LIST_DIR}/ACMP_Wakeup/main.c)
create_project(ACMP_WindowCompare ${CMAKE_CURRENT_LIST_DIR}/ACMP_WindowCompare/main.c)
create_project(CANFD_CANFD_Loopback ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_Loopback/main.c)
create_project(CANFD_CANFD_TxRx ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_TxRx/main.c)
create_project(CANFD_CANFD_TxRxINT ${CMAKE_CURRENT_LIST_DIR}/CANFD_CANFD_TxRxINT/main.c)

# create_project(CANFD_CAN_Loopback ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_Loopback/main.c)
# create_project(CANFD_CAN_TxRx ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_TxRx/main.c)
# create_project(CANFD_CAN_TxRxINT ${CMAKE_CURRENT_LIST_DIR}/CANFD_CAN_TxRxINT/main.c)
create_project(CLK_ClockDetector ${CMAKE_CURRENT_LIST_DIR}/CLK_ClockDetector/main.c)

# create_project(CLK_ClockDetectorWakeup ${CMAKE_CURRENT_LIST_DIR}/CLK_ClockDetectorWakeup/main.c)
# create_project(FMC_APWPROT ${CMAKE_CURRENT_LIST_DIR}/FMC_APWPROT/main.c)
create_project(FMC_CRC32 ${CMAKE_CURRENT_LIST_DIR}/FMC_CRC32/main.c)
create_project(FMC_DualBank ${CMAKE_CURRENT_LIST_DIR}/FMC_DualBank/main.c)
add_subdirectory(FMC_DualBankFwUpdate)
create_project(FMC_ExeInSRAM ${CMAKE_CURRENT_LIST_DIR}/FMC_ExeInSRAM/main.c ${CMAKE_CURRENT_LIST_DIR}/FMC_ExeInSRAM/exeinsram.c)
add_subdirectory(FMC_FwUpdateApplication)
add_subdirectory(FMC_IAP)
create_project(FMC_MultiBoot ${CMAKE_CURRENT_LIST_DIR}/FMC_MultiBoot/main.c)

# create_project(FMC_MultiWordProgram ${CMAKE_CURRENT_LIST_DIR}/FMC_MultiWordProgram/main.c)
create_project(FMC_ReadAllOne ${CMAKE_CURRENT_LIST_DIR}/FMC_ReadAllOne/main.c)
create_project(FMC_RW ${CMAKE_CURRENT_LIST_DIR}/FMC_RW/main.c)

# create_project(FMC_SPROM ${CMAKE_CURRENT_LIST_DIR}/FMC_SPROM/main.c ${CMAKE_CURRENT_LIST_DIR}/FMC_SPROM/sprom.c)
create_project(GPIO_EINTAndDebounce ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTAndDebounce/main.c)

# create_project(GPIO_EINTTriggerPDMA ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerPDMA/main.c)
# create_project(GPIO_EINTTriggerPWM ${CMAKE_CURRENT_LIST_DIR}/GPIO_EINTTriggerPWM/main.c)
create_project(GPIO_INT ${CMAKE_CURRENT_LIST_DIR}/GPIO_INT/main.c)
create_project(GPIO_OutputInput ${CMAKE_CURRENT_LIST_DIR}/GPIO_OutputInput/main.c)
create_project(GPIO_PowerDown ${CMAKE_CURRENT_LIST_DIR}/GPIO_PowerDown/main.c)

# create_project(LLSI_Marquee ${CMAKE_CURRENT_LIST_DIR}/LLSI_Marquee/main.c)
# create_project(LLSI_PDMA_Marquee ${CMAKE_CURRENT_LIST_DIR}/LLSI_PDMA_Marquee/main.c)
create_project(PDMA_BasicMode ${CMAKE_CURRENT_LIST_DIR}/PDMA_BasicMode/main.c)
create_project(PDMA_ScatterGather ${CMAKE_CURRENT_LIST_DIR}/PDMA_ScatterGather/main.c)
create_project(PDMA_ScatterGather_PingPongBuffer ${CMAKE_CURRENT_LIST_DIR}/PDMA_ScatterGather_PingPongBuffer/main.c)
create_project(SPI_Loopback ${CMAKE_CURRENT_LIST_DIR}/SPI_Loopback/main.c)
create_project(SPI_MasterFIFOMode ${CMAKE_CURRENT_LIST_DIR}/SPI_MasterFIFOMode/main.c)
create_project(SPI_SlaveFIFOMode ${CMAKE_CURRENT_LIST_DIR}/SPI_SlaveFIFOMode/main.c)
create_project(SYS_BODWakeup ${CMAKE_CURRENT_LIST_DIR}/SYS_BODWakeup/main.c)
create_project(SYS_PLLClockOutput ${CMAKE_CURRENT_LIST_DIR}/SYS_PLLClockOutput/main.c)

# create_project(SYS_PowerDown_MinCurrent ${CMAKE_CURRENT_LIST_DIR}/SYS_PowerDown_MinCurrent/main.c)
create_project(SYS_TrimHIRC ${CMAKE_CURRENT_LIST_DIR}/SYS_TrimHIRC/main.c)
create_project(UART_AutoBaudRate ${CMAKE_CURRENT_LIST_DIR}/UART_AutoBaudRate/main.c)
create_project(UART_Autoflow ${CMAKE_CURRENT_LIST_DIR}/UART_Autoflow/main.c)
create_project(UART_IrDA ${CMAKE_CURRENT_LIST_DIR}/UART_IrDA/main.c)
create_project(UART_LIN ${CMAKE_CURRENT_LIST_DIR}/UART_LIN/main.c)

# create_project(UART_LIN_Wakeup ${CMAKE_CURRENT_LIST_DIR}/UART_LIN_Wakeup/main.c)
create_project(UART_PDMA ${CMAKE_CURRENT_LIST_DIR}/UART_PDMA/main.c)
create_project(UART_RS485 ${CMAKE_CURRENT_LIST_DIR}/UART_RS485/main.c)
create_project(UART_SingleWire ${CMAKE_CURRENT_LIST_DIR}/UART_SingleWire/main.c)
create_project(UART_TxRxFunction ${CMAKE_CURRENT_LIST_DIR}/UART_TxRxFunction/main.c)
create_project(UART_Wakeup ${CMAKE_CURRENT_LIST_DIR}/UART_Wakeup/main.c)
create_project(USCI_SPI_Loopback ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_Loopback/main.c)
create_project(USCI_SPI_MasterMode ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_MasterMode/main.c)

# create_project(USCI_SPI_PDMA_LoopTest ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_PDMA_LoopTest/main.c)
create_project(USCI_SPI_SlaveMode ${CMAKE_CURRENT_LIST_DIR}/USCI_SPI_SlaveMode/main.c)
create_project(USCI_UART_AutoBaudRate ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_AutoBaudRate/main.c)
create_project(USCI_UART_AutoFlow ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_AutoFlow/main.c)
create_project(USCI_UART_PDMA ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_PDMA/main.c)
create_project(USCI_UART_RS485 ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_RS485/main.c)
create_project(USCI_UART_TxRxFunction ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_TxRxFunction/main.c)
create_project(USCI_UART_Wakeup ${CMAKE_CURRENT_LIST_DIR}/USCI_UART_Wakeup/main.c)
create_project(QSPI_DualMode_Flash ${CMAKE_CURRENT_LIST_DIR}/QSPI_DualMode_Flash/main.c)
create_project(QSPI_QuadMode_Flash ${CMAKE_CURRENT_LIST_DIR}/QSPI_QuadMode_Flash/main.c)
create_project(QSPI_Slave3Wire ${CMAKE_CURRENT_LIST_DIR}/QSPI_Slave3Wire/main.c)

create_project(USBD_VCOM_MultiPort ${CMAKE_CURRENT_LIST_DIR}/USBD_VCOM_MultiPort/main.c ${CMAKE_CURRENT_LIST_DIR}/USBD_VCOM_MultiPort/descriptors.c ${CMAKE_CURRENT_LIST_DIR}/USBD_VCOM_MultiPort/vcom_serial.c)