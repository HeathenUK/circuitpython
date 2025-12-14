// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include "py/runtime.h"

#include "common-hal/esphosted/__init__.h"
#include "shared-bindings/microcontroller/Pin.h"

#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Include board configuration for pin definitions
#include "mpconfigboard.h"

static const char *TAG = "CP esphosted";

static bool esp_hosted_initialized = false;

// Board-defined pin configuration
#ifdef CIRCUITPY_ESP_HOSTED_PINS
static const esphosted_pins_t esp_hosted_pins = CIRCUITPY_ESP_HOSTED_PINS;
#endif

const esphosted_pins_t *common_hal_esphosted_get_pins(void) {
    #ifdef CIRCUITPY_ESP_HOSTED_PINS
    return &esp_hosted_pins;
    #else
    return NULL;
    #endif
}

bool common_hal_esphosted_is_initialized(void) {
    return esp_hosted_initialized;
}

void common_hal_esphosted_reset_coprocessor(void) {
    #ifdef CIRCUITPY_ESP_HOSTED_PINS
    const mcu_pin_obj_t *reset_pin = esp_hosted_pins.reset;

    if (reset_pin == NULL) {
        ESP_LOGW(TAG, "No reset pin configured");
        return;
    }

    ESP_LOGI(TAG, "Resetting co-processor via GPIO%d", reset_pin->number);

    gpio_reset_pin(reset_pin->number);
    gpio_set_direction(reset_pin->number, GPIO_MODE_OUTPUT);

    // Assert reset (active low)
    gpio_set_level(reset_pin->number, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Release reset
    gpio_set_level(reset_pin->number, 1);

    // Wait for co-processor to boot and initialize
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Co-processor reset complete");
    #endif
}

void common_hal_esphosted_init(void) {
    #ifdef CIRCUITPY_ESP_HOSTED_PINS
    if (esp_hosted_initialized) {
        return;
    }

    ESP_LOGI(TAG, "Initializing ESP-Hosted");

    // Validate required pins
    if (esp_hosted_pins.clk == NULL ||
        esp_hosted_pins.cmd == NULL ||
        esp_hosted_pins.d0 == NULL) {
        ESP_LOGE(TAG, "Missing required SDIO pins (clk, cmd, d0)");
        mp_raise_RuntimeError(MP_ERROR_TEXT("ESP-Hosted: missing required SDIO pins"));
        return;
    }

    // Reset the co-processor first
    common_hal_esphosted_reset_coprocessor();

    // Determine bus width based on configured pins
    int bus_width = 1;
    if (esp_hosted_pins.d1 != NULL &&
        esp_hosted_pins.d2 != NULL &&
        esp_hosted_pins.d3 != NULL) {
        bus_width = 4;
    }

    ESP_LOGI(TAG, "Configuring SDIO with %d-bit bus width", bus_width);
    ESP_LOGI(TAG, "  CLK: GPIO%d", esp_hosted_pins.clk->number);
    ESP_LOGI(TAG, "  CMD: GPIO%d", esp_hosted_pins.cmd->number);
    ESP_LOGI(TAG, "  D0:  GPIO%d", esp_hosted_pins.d0->number);
    if (bus_width == 4) {
        ESP_LOGI(TAG, "  D1:  GPIO%d", esp_hosted_pins.d1->number);
        ESP_LOGI(TAG, "  D2:  GPIO%d", esp_hosted_pins.d2->number);
        ESP_LOGI(TAG, "  D3:  GPIO%d", esp_hosted_pins.d3->number);
    }
    if (esp_hosted_pins.reset != NULL) {
        ESP_LOGI(TAG, "  RST: GPIO%d", esp_hosted_pins.reset->number);
    }
    if (esp_hosted_pins.data_ready != NULL) {
        ESP_LOGI(TAG, "  DR:  GPIO%d", esp_hosted_pins.data_ready->number);
    }

    // Configure SDIO host
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  // 40MHz

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = bus_width;
    slot_config.clk = esp_hosted_pins.clk->number;
    slot_config.cmd = esp_hosted_pins.cmd->number;
    slot_config.d0 = esp_hosted_pins.d0->number;

    if (bus_width == 4) {
        slot_config.d1 = esp_hosted_pins.d1->number;
        slot_config.d2 = esp_hosted_pins.d2->number;
        slot_config.d3 = esp_hosted_pins.d3->number;
    }

    // Initialize SDIO host controller
    esp_err_t err = sdmmc_host_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SDMMC host: 0x%x", err);
        mp_raise_RuntimeError(MP_ERROR_TEXT("ESP-Hosted: SDMMC host init failed"));
        return;
    }

    err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_0, &slot_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SDMMC slot: 0x%x", err);
        sdmmc_host_deinit();
        mp_raise_RuntimeError(MP_ERROR_TEXT("ESP-Hosted: SDMMC slot init failed"));
        return;
    }

    // Configure data ready GPIO as input if defined
    if (esp_hosted_pins.data_ready != NULL) {
        gpio_num_t dr_pin = esp_hosted_pins.data_ready->number;
        gpio_reset_pin(dr_pin);
        gpio_set_direction(dr_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(dr_pin, GPIO_PULLDOWN_ONLY);
    }

    // TODO: Initialize ESP-Hosted protocol layer
    // When the esp_hosted component is integrated, uncomment and add:
    //
    // #include "esp_hosted_api.h"  // Add to includes at top of file
    //
    // esp_hosted_config_t hosted_config = {
    //     .transport = ESP_HOSTED_TRANSPORT_SDIO,
    //     .sdio = {
    //         .host = host,
    //         .slot = SDMMC_HOST_SLOT_0,
    //     },
    //     .data_ready_gpio = esp_hosted_pins.data_ready ?
    //                        esp_hosted_pins.data_ready->number : -1,
    // };
    //
    // err = esp_hosted_init(&hosted_config);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize ESP-Hosted: 0x%x", err);
    //     sdmmc_host_deinit();
    //     mp_raise_RuntimeError(MP_ERROR_TEXT("ESP-Hosted: protocol init failed"));
    //     return;
    // }
    //
    // // Initialize WiFi on the hosted interface
    // err = esp_hosted_wifi_init();
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize ESP-Hosted WiFi: 0x%x", err);
    //     return;
    // }

    esp_hosted_initialized = true;
    ESP_LOGI(TAG, "ESP-Hosted SDIO transport initialized");
    ESP_LOGW(TAG, "Note: Full WiFi support requires esp_hosted component integration");
    #else
    ESP_LOGE(TAG, "ESP-Hosted pins not configured for this board");
    mp_raise_RuntimeError(MP_ERROR_TEXT("ESP-Hosted: no pin configuration"));
    #endif
}

void common_hal_esphosted_deinit(void) {
    if (!esp_hosted_initialized) {
        return;
    }

    ESP_LOGI(TAG, "Deinitializing ESP-Hosted");

    // TODO: Deinitialize ESP-Hosted protocol layer
    // esp_hosted_deinit();

    sdmmc_host_deinit();

    esp_hosted_initialized = false;
}
