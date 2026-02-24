=============================
Raspberry Pi 5 RP1 PINCTRL
=============================

Overview
========

This sample validates the **RP1 pinctrl** driver on **Raspberry Pi 5** by cycling a GPIO
pin through alternate functions (ALT0..ALT5) and reading back the RP1 CTRL register.

Location
========

- ``samples/rpi5_pinctrl``

Behavior
========

- Uses Devicetree node ``rp1_pinctrl``
- Cycles ``GPIO17`` (default) through ``ALT0``..``ALT5`` every 1 second
- Prints the CTRL register value after each change (e.g., ``0x00000080`` .. ``0x00000085``)

Build
=====

.. code-block:: bash

   west build -b rpi_5 samples/rpi5_pinctrl -p

Expected output
===============

You should see RP1 pinctrl initialization/mapping logs and then the mode-cycling test:

.. code-block:: text

   RP1_PINCTRL_INIT: dev=0x213300
   [00:00:00.029,000] <inf> pinctrl_rp1: RP1 pinctrl mapped: gpio=0x9e9000 pads=0x9e7000
   *** Booting Zephyr OS build vX.Y.Z-... ***

   === RP1 PINCTRL MODE CYCLING TEST ===
   Setting GPIO17 -> ALT0
   CTRL = 0x00000080
   Setting GPIO17 -> ALT1
   CTRL = 0x00000081
   Setting GPIO17 -> ALT2
   CTRL = 0x00000082
   Setting GPIO17 -> ALT3
   CTRL = 0x00000083
   Setting GPIO17 -> ALT4
   CTRL = 0x00000084
   Setting GPIO17 -> ALT5
   CTRL = 0x00000085

Notes
=====

- Change ``TEST_PIN`` in ``src/main.c`` to test a different GPIO.
- If you see ``Device not ready``, ensure ``rp1_pinctrl`` exists and is enabled in Devicetree.
- If debug logs are too noisy, reduce log level or disable OS debug logging in your config.