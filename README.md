# HALMET Example Firmware

This repository provides example firmware for [HALMET: Hat Labs Marine Engine & Tank interface](https://shop.hatlabs.fi/products/halmet).

To get started with the example firmware, follow the generic SensESP [Getting Started](https://signalk.org/SensESP/pages/getting_started/) instructions but use this repository instead of the SensESP Project Template.

By default, the example firmware is configured to read the engine RPM from input D1 and the fuel level from input A1. D2 is configured as a low oil pressure alarm input.

To customize the software for your own purposes, edit the `src/main.cpp` file.
Parts intended to be customized are marked with `EDIT:` comments.

## Building and flashing

`pio run` builds the `halmet_espidf` environment, which compiles ESP-IDF from source so that `sdkconfig.defaults` takes effect. This is the build to flash: it enables the dynamic mbedTLS buffers that let the device hold a TLS connection to a Signal K server. The first build downloads ESP-IDF (several hundred megabytes) and takes several minutes; later builds are incremental.

The `halmet` environment (`pio run -e halmet`) uses the precompiled Arduino libraries and builds in a fraction of the time. Use it for quick compile checks only. A device flashed with it boots and joins WiFi, but the Signal K connection stays disconnected against a TLS server because `mbedtls_ssl_setup` runs out of memory.

On Windows, keep the project on a short path without spaces; the ESP-IDF build fails on long paths.

Memory-tight builds can set `CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN=8192` in `sdkconfig.defaults` to free about 8 KB per TLS connection. The cost is that any inbound TLS record larger than 8 KB is rejected, which breaks the handshake against servers with long certificate chains; this example keeps the 16 KB default. After editing `sdkconfig.defaults`, delete the generated `sdkconfig.halmet_espidf` in the project root: it overrides the defaults and is regenerated on the next build.

The tacho input uses the ESP32 pulse counter (PCNT) peripheral through `DigitalInputPcntCounter`; this example targets the original ESP32 that HALMET carries and does not build for other ESP32 variants without changes.

## Upgrading from an earlier version

The environment names changed meaning. `halmet` used to be the build to flash; it is now the Arduino compile-check build described above, and `halmet_espidf` is what you flash. Plain `pio run` and `pio run -t upload` pick `halmet_espidf` automatically.

A previously generated `sdkconfig.halmet_espidf` in the project root overrides `sdkconfig.defaults` and survives `pio run -t fullclean`. After pulling a change to `sdkconfig.defaults`, delete `sdkconfig.halmet_espidf`; it is regenerated on the next build.

A device running an earlier Arduino build needs one flash over USB before it can take the `halmet_espidf` application over the air. The partition table (`default_8MB.csv`) is unchanged, so the WiFi and Signal K settings stored on the device survive the USB flash.
