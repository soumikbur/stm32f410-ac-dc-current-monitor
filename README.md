# STM32F410 FreeRTOS AC/DC Current Monitoring System

![STM32](https://img.shields.io/badge/MCU-STM32F410RBT6-blue)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)
![Language](https://img.shields.io/badge/Language-C-orange)
![ADC](https://img.shields.io/badge/Peripheral-ADC%20DMA-red)

## Overview

A real-time AC/DC current monitoring system developed on the **STM32F410RBT6** microcontroller using **FreeRTOS**, **ADC**, and **DMA**.

The firmware continuously samples analog current sensor data, performs digital signal processing, and computes current values in real time while maintaining deterministic task scheduling through FreeRTOS.

This project demonstrates:

* Real-time embedded system design
* FreeRTOS task management
* ADC sampling with DMA
* Current signal acquisition
* Multi-task synchronization
* STM32 HAL-based firmware architecture

---

## Hardware Platform

### Microcontroller

| Parameter       | Value         |
| --------------- | ------------- |
| MCU             | STM32F410RBT6 |
| Core            | ARM Cortex-M4 |
| Clock Frequency | Up to 100 MHz |
| Flash           | 128 KB        |
| SRAM            | 32 KB         |

### Peripherals Used

* ADC1
* DMA
* UART
* GPIO
* FreeRTOS Scheduler
* SysTick Timer

---

## System Architecture

```text
                    +-------------------+
                    | Current Sensor    |
                    +---------+---------+
                              |
                              v

                    +-------------------+
                    | ADC Sampling      |
                    +---------+---------+
                              |
                              v

                    +-------------------+
                    | DMA Transfer      |
                    +---------+---------+
                              |
                              v

                    +-------------------+
                    | FreeRTOS Tasks    |
                    +---------+---------+
                              |
              +---------------+---------------+
              |                               |
              v                               v

     +----------------+             +----------------+
     | Signal Process |             | UART Output    |
     +--------+-------+             +--------+-------+
              |                              |
              +--------------+---------------+
                             |
                             v

                    +-------------------+
                    | Current Value     |
                    | Monitoring        |
                    +-------------------+
```

---

## Software Architecture

```text
+------------------------------------------------+
|                 FreeRTOS Kernel                |
+------------------------------------------------+
          |                     |
          |                     |
          v                     v

+----------------+     +----------------------+
| ADC Task       |     | Processing Task      |
+----------------+     +----------------------+

          |                     |
          +----------+----------+
                     |
                     v

            +------------------+
            | UART Task        |
            +------------------+
```

---

## Project Structure

```text
STM32F410-AC-DC-Current-Monitor
│
├── Core
│   ├── Inc
│   └── Src
│
├── Drivers
│   ├── CMSIS
│   └── STM32F4xx_HAL_Driver
│
├── Middlewares
│   └── FreeRTOS
│
├── STM32F410RBTX_FLASH.ld
│
├── .ioc
│
└── README.md
```

---

## Features

### Real-Time Operation

* FreeRTOS-based scheduling
* Deterministic task execution
* Efficient CPU utilization

### ADC Acquisition

* Continuous sampling mode
* DMA-assisted data transfer
* Reduced CPU overhead

### Signal Processing

* Analog current acquisition
* Real-time current computation
* Noise reduction through averaging

### Communication

* UART debugging output
* Monitoring and diagnostics support

---

## Development Environment

| Tool         | Version          |
| ------------ | ---------------- |
| STM32CubeIDE | Latest           |
| STM32CubeMX  | Integrated       |
| FreeRTOS     | CMSIS-RTOS V2    |
| Compiler     | GCC ARM Embedded |

---

## Build Instructions

### Clone Repository

```bash
git clone https://github.com/soumikbur/stm32f410-ac-dc-current-monitor.git
```

### Open Project

1. Launch STM32CubeIDE
2. Import Existing Project
3. Select repository folder
4. Build project

### Flash Firmware

1. Connect ST-LINK
2. Build project
3. Run → Debug
4. Program STM32F410RBT6

---

## Current Data Flow

```text
Sensor Input
      |
      v
ADC Conversion
      |
      v
DMA Buffer
      |
      v
Processing Task
      |
      v
Current Calculation
      |
      v
UART Monitoring
```

---

## Applications

* Industrial current monitoring
* Embedded instrumentation
* Energy measurement systems
* Real-time data acquisition
* Embedded RTOS learning
* STM32 firmware development

---

## Future Improvements

* True RMS current calculation
* OLED/LCD display support
* Modbus communication
* Data logging
* MQTT/IoT integration
* Calibration interface

---

## Author

**Soumik Ghosh**

Embedded Software Developer

GitHub:
https://github.com/soumikbur

---

## License

This project is released under the MIT License.
