# STM32F103RB-Nucleo
STM32F103RB Nucleo Dev Board peripherals tests

This repository contains three test projects demonstrating the use of UART and SPI peripherals on the STM32F103RB Nucleo board. The tests include configurations for interrupts and DMA to efficiently handle data transfers. Each project focuses on different communication techniques and peripheral configurations commonly used in embedded systems.

Tests Included:
1. UART with Interrupts
This test demonstrates asynchronous communication using the UART peripheral with interrupts. The data transmission and reception are handled through interrupt service routines (ISR), ensuring efficient handling of serial communication without polling.
Key features:
UART initialization and configuration.
Interrupt-driven transmission and reception.
Handling data through interrupt routines.
Demonstrates non-blocking UART communication.
2. UART with DMA
This test utilizes Direct Memory Access (DMA) for UART data transfers. DMA allows continuous and high-speed data transfer between memory and the UART peripheral, reducing CPU load by offloading data transfer tasks.
Key features:
UART with DMA for data transfer.
DMA initialization for both transmission and reception.
Non-blocking data transfer using DMA, freeing the CPU for other tasks.
3. SPI with DMA
In this test, the SPI peripheral is configured for communication using DMA. Similar to the UART DMA example, DMA handles the data transfer between the memory and SPI peripheral, making this an efficient method for handling large amounts of data in SPI communication.
Key features:
SPI configuration for master mode.
DMA initialization for SPI data transfer.
Non-blocking communication between the SPI master and the slave using DMA.
