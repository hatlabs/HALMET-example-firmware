# HALMET Example Firmware

This repository provides example firmware for [HALMET: Hat Labs Marine Engine & Tank interface](https://shop.hatlabs.fi/products/halmet).

To get started with the example firmware, follow the generic SensESP [Getting Started](https://signalk.org/SensESP/pages/getting_started/) instructions but use this repository instead of the SensESP Project Template.

By default, the example firmware is configured to read the engine RPM from input D1 and the fuel level from input A1. D2 is configured as a low oil pressure alarm input.

To customize the software for your own purposes, edit the `src/main.cpp` file.
Parts intended to be customized are marked with `EDIT:` comments.
