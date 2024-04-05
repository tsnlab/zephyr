.. rpi_5:

Raspberry Pi 5 (Cortex-A76)
###########################

Overview
********

`Raspberry Pi 5 product-brief`_

Hardware
********

- Broadcom BCM2712 2.4GHz quad-core 64-bit Arm Cortex-A76 CPU, with cryptography extensions, 512KB per-core L2 caches and a 2MB shared L3 cache
- VideoCore VII GPU, supporting OpenGL ES 3.1, Vulkan 1.2
- Dual 4Kp60 HDMI® display output with HDR support
- 4Kp60 HEVC decoder
- LPDDR4X-4267 SDRAM (4GB and 8GB SKUs available at launch)
- Dual-band 802.11ac Wi-Fi®
- Bluetooth 5.0 / Bluetooth Low Energy (BLE)
- microSD card slot, with support for high-speed SDR104 mode
- 2 x USB 3.0 ports, supporting simultaneous 5Gbps operation
- 2 x USB 2.0 ports
- Gigabit Ethernet, with PoE+ support (requires separate PoE+ HAT)
- 2 x 4-lane MIPI camera/display transceivers
- PCIe 2.0 x1 interface for fast peripherals (requires separate M.2 HAT or other adapter)
- 5V/5A DC power via USB-C, with Power Delivery support
- Raspberry Pi standard 40-pin header
- Real-time clock (RTC), powered from external battery
- Power button

Supported Features
==================

The Raspberry Pi 5 board configuration supports the following hardware features:

+-----------+---------------------+--------------------------+
| Interface | Kconfig             | Devicetree compatible    |
+===========+=====================+==========================+
| GIC-400   | N/A                 | arm,gic-v2               |
+-----------+---------------------+--------------------------+
| GPIO      | CONFIG_GPIO         | brcm,brcmstb-gpio        |
+-----------+---------------------+--------------------------+
| SERIAL    | CONFIG_SERIAL       | arm,pl011                |
|           | CONFIG_CONSOLE      |                          |
|           | CONFIG_UART_CONSOLE |                          |
+-----------+---------------------+--------------------------+

There are many hardware fatures of Raspberry Pi 5. But now, `GPIO` and `SERIAL` feautres have been implemented for Raspberry Pi 5.

Other hardware features are not currently supported by the port. See the `Raspberry Pi hardware`_ for a complete list of MPS3 AN547 board hardware features.

The default configuration can be found in
:zephyr_file:`boards/raspberrypi/rpi_5/rpi_5_defconfig`.

Programming and Debugging
*************************

Flashing
========

In brief,
    1. Format your Micro SD card with MBR and FAT32.
    2. Save three files below in the root directory.
        * config.txt
        * zephyr.bin
        * `bcm2712-rpi-5.dtb`_
    3. Insert the Micro SD card and power on the Raspberry Pi 5.

then, You will see the Raspberry Pi 5 running the `zephyr.bin`.

config.txt
----------

.. code-block:: text

   kernel=zephyr.bin
   arm_64bit=1


zephyr.bin
----------

Build an app `samples/basic/blinky`

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: rpi_5
   :goals: build

Copy `zephyr.bin` from `build/zephyr` directory to the root directory of the Micro SD card.

Insert the Micro SD card and power on the Raspberry Pi 5. And then, the STAT LED will start to blink.


Serial Communication
====================

wiring
------

you will need to prepare the following items:
   * `Raspberry Pi Debug Probe`_
   * JST cable: 3-pin JST connector to 3-pin JST connector cable
   * USB cable: USB A male - Micro USB B male

Use a JST cable to connect the Raspberry Pi Debug Probe UART port and the Raspberry Pi 5 UART port between the HDMI ports.

Then connect the Raspberry Pi Debug Probe and your computer with a USB cable.


config.txt
----------

.. code-block:: text

   kernel=zephyr.bin
   arm_64bit=1
   enable_uart=1
   uart_2ndstage=1


zephyr.bin
----------

Build an app `samples/hello_world`

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rpi_5
   :goals: build

Copy `zephyr.bin` from `build/zephyr` directory to the root directory of the Micro SD card.

Insert the Micro SD card into your Raspberry Pi 5.


serial terminal emulator
------------------------

Set the baud rate to `115200` and the serial device(or line) to `/dev/ttyACM0`.

When you power on the Raspberry Pi 5, you will see the following output in the serial console:

.. code-block:: console

   *** Booting Zephyr OS build XXXXXXXXXXXX  ***
   Hello World! rpi_5/bcm2712


.. _Raspberry Pi 5 product-brief:
   https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf

.. _Raspberry Pi hardware:
   https://www.raspberrypi.com/documentation/computers/raspberry-pi.html

.. _bcm2712-rpi-5.dtb:
   https://github.com/raspberrypi/firmware/raw/master/boot/bcm2712-rpi-5-b.dtb

.. _Raspberry Pi Debug Probe:
   https://www.raspberrypi.com/products/debug-probe/
