// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

#include "py/obj.h"
#include "shared-bindings/microcontroller/Pin.h"

// ESP-Hosted pin configuration structure
// Used to define the SDIO connection to the WiFi co-processor
typedef struct {
    const mcu_pin_obj_t *clk;        // SDIO clock
    const mcu_pin_obj_t *cmd;        // SDIO command
    const mcu_pin_obj_t *d0;         // SDIO data 0
    const mcu_pin_obj_t *d1;         // SDIO data 1 (NULL for 1-bit mode)
    const mcu_pin_obj_t *d2;         // SDIO data 2 (NULL for 1-bit mode)
    const mcu_pin_obj_t *d3;         // SDIO data 3 (NULL for 1-bit mode)
    const mcu_pin_obj_t *reset;      // Co-processor reset pin (active low)
    const mcu_pin_obj_t *data_ready; // Data ready signal from co-processor
} esphosted_pins_t;

// Initialize ESP-Hosted subsystem using board-defined pins
// This should be called before wifi initialization on boards that use ESP-Hosted
void common_hal_esphosted_init(void);

// Deinitialize ESP-Hosted subsystem
void common_hal_esphosted_deinit(void);

// Check if ESP-Hosted has been initialized
bool common_hal_esphosted_is_initialized(void);

// Reset the co-processor
void common_hal_esphosted_reset_coprocessor(void);

// Get the pin configuration (returns NULL if not configured)
const esphosted_pins_t *common_hal_esphosted_get_pins(void);
