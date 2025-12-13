// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Nicolai Electronics
//
// SPDX-License-Identifier: MIT
//

#pragma once

// Micropython setup

#define MICROPY_HW_BOARD_NAME       "Tanmatsu"
#define MICROPY_HW_MCU_NAME         "ESP32P4"

// I2C bus
#define CIRCUITPY_BOARD_I2C         (1)
#define CIRCUITPY_BOARD_I2C_PIN     {{.scl = &pin_GPIO10, .sda = &pin_GPIO9}}

// Use the second USB device (numbered 0 and 1)
#define CIRCUITPY_USB_DEVICE_INSTANCE 0
#define CIRCUITPY_ESP32P4_SWAP_LSFS (1)
