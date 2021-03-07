ESP-IDF app for Spartan Edge Accelerator
====================

A messy app based on the ESP-IDF app template. This is meant to be run on ESP32 that is part of the Spartan Edge Accelerator board.

The application repeatedly sends data using the ESP32's SPI peripheral configured for QSPI and sending using DMA to minimize CPU usage.

The application waits for a GPIO interrupt which is meant to signal the beginning of the VBLANK period on a hardware design specified in [this](https://github.com/smartperson/spartan-edge-accelerator-graphical-system) repository.

The goal is to ultimately allow for the creation of rich, graphically complex applications on the ESP32, with the FPGA handling the graphics processing. The sibling repository has more information, and will be used to track most of the active development on this project.
