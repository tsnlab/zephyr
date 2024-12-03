.. _topst_vcp45:

TOPST_VCP45
###################

Overview
********

The TOPST_VCP45 is a 32-Bit MCU Board for Real time Applications.

TOPST VCP can implement the following functions:

- Automotive Application
    Car Audio Video Navigation (AVN)System, Digital Cluster, Head Up Display (HUD), etc.
- IOT Application
    Smart Factory, etc.
- Training & Education
    Sensor, Actuator Application, CAN Application, etc.

TOPST: Total Open-Platform for System development and Training
VCP: Vehicle Control Processor

.. image:: topst_vcp45.png
     :align: center
     :alt: TOPST_VCP45 Board

Hardware
********

- ARM Cortex-R5F Processor
- On-chip Memory
    Program Flash: 4 MB
	SRAM         : 512KB (Including Retention RAM 16KB)
    Data Flash   : 128KB
    DMA Channel  : 22-channel
- Peripheral
    CAN/CANFD                       : 3-channel
    Dedicated LIN/UART              : 3-channel (Maximum 6-channel)
    Dedicated I2C                   : 3-channel (Maximum 6-channel)
    Dedicated GPSB (SPI)            : 2-channel (Maximum 5-channel)
    MFIO (Allocated UART, I2C, GPSB): 3-channel
    ADC
	    Resolution : 12-bit SAR type
        Channels   : 12-channel x 2groups
        Input Range: 3.3V
        Sample Rate: Over 1.0 MSPs
    I2S                   : 1-channel
    Serial Flash Interface: Quad SPI

Supported Features
==================

The following features are supported:

.. list-table::
   :header-rows: 2

   * - Peripheral
     - Kconfig option
     - Devicetree compatible
   * - TIC
     - :kconfig:option:`CONFIG_TIC`
     - :dtcompatible:`tcc,tic`
   * - GPIO
     - :kconfig:option:`CONFIG_GPIO` & 'GPIO_TCCVCP'
     - :dtcompatible:`tcc,tccvcp-gpio`
   * - UART
     - :kconfig:option:`CONFIG_SERIAL` & 'CONFIG_UART_TCCVCP'
     - :dtcompatible:`tcc,tccvcp-uart`
   * - Timer
     - :kconfig:option:`CONFIG_TCC_VCPTTC_TIMER`
     - :dtcompatible:`tcc,ttcvcp`
   * - Clock
     - :kconfig:option:`CONFIG_CLOCK_CONTROL_TCC_CCU`
     - :dtcompatible:`tcc,ccu`

Not all hardware features are supported yet. See `TCC70xx Full Specification`_ for the complete list of hardware features.

The default configuration can be found in
:zephyr_file:`boards/tcc/topst_vcp_45/topst_vcp_45_defconfig`

Programming and Debugging
*************************

hello_world
===========

In brief,
    1. Format your Micro SD card with MBR and FAT32.
    2. Save three files below in the root directory.
        * config.txt
        * zephyr.bin
        * `bcm2712-rpi-5.dtb`_
    3. Insert the Micro SD card and power on the Raspberry Pi 5.

then, You will see the Raspberry Pi 5 running the :file:`zephyr.bin`.

config.txt
----------

.. code-block:: text

   kernel=zephyr.bin
   arm_64bit=1


zephyr.bin
----------

Build an app, for example :zephyr:code-sample:`blinky`

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: rpi_5
   :goals: build

Copy :file:`zephyr.bin` from :file:`build/zephyr` directory to the root directory of the Micro SD
card.

Insert the Micro SD card and power on the Raspberry Pi 5. And then, the STAT LED will start to blink.



Flashing
========


Debugging
=========


References
**********
