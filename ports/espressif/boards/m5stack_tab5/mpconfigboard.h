// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2019 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

// Micropython setup

#define MICROPY_HW_BOARD_NAME       "M5Stack Tab5"
#define MICROPY_HW_MCU_NAME         "ESP32P4"

// I2C bus for touch, IMU, RTC, power monitor, and GPIO expanders
#define CIRCUITPY_BOARD_I2C         (1)
#define CIRCUITPY_BOARD_I2C_PIN     {{.scl = &pin_GPIO32, .sda = &pin_GPIO31}}

// UART
#define DEFAULT_UART_BUS_RX         (&pin_GPIO38)
#define DEFAULT_UART_BUS_TX         (&pin_GPIO37)

// SPI on M5-Bus
#define DEFAULT_SPI_BUS_SCK         (&pin_GPIO5)
#define DEFAULT_SPI_BUS_MOSI        (&pin_GPIO18)
#define DEFAULT_SPI_BUS_MISO        (&pin_GPIO19)

// ESP-Hosted co-processor (ESP32-C6 connected via SDIO)
// Directly connected to ESP32-P4's second SDMMC slot
#define CIRCUITPY_ESP_HOSTED_PINS { \
    .clk = &pin_GPIO12, \
    .cmd = &pin_GPIO13, \
    .d0 = &pin_GPIO11, \
    .d1 = &pin_GPIO10, \
    .d2 = &pin_GPIO9, \
    .d3 = &pin_GPIO8, \
    .reset = &pin_GPIO15, \
    .data_ready = &pin_GPIO14, \
}

// Use the second USB device (numbered 0 and 1)
#define CIRCUITPY_USB_DEVICE_INSTANCE 0
#define CIRCUITPY_ESP32P4_SWAP_LSFS (1)
