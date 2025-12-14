// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Nicolai Electronics
//
// SPDX-License-Identifier: MIT
//
// This board definition is based on the Tanmatsu hardware design and firmware.
// Sources and references:
// - Badge Team ESP32 Component Badge BSP: https://github.com/badgeteam/esp32-component-badge-bsp
//   (Hardware definitions, pin mappings, and coprocessor register maps)
// - Tanmatsu hardware documentation and schematics at https://docs.tanmatsu.cloud
// - M5Stack Tab5 board definition (base reference for ESP32-P4 port) by tannewt

#include "supervisor/board.h"
#include "mpconfigboard.h"
#include "shared-bindings/board/__init__.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/mipidsi/Bus.h"
#include "shared-bindings/mipidsi/Display.h"
#include "shared-module/displayio/__init__.h"
#include "shared-module/framebufferio/__init__.h"
#include "shared-module/framebufferio/FramebufferDisplay.h"
#include "ports/espressif/common-hal/microcontroller/Pin.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "py/mphal.h"
#include "py/ringbuf.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "py/nlr.h"
#include "supervisor/shared/serial.h"
#include "shared/runtime/interrupt_char.h"
#include "supervisor/background_callback.h"
#include "supervisor/shared/tick.h"
#include "supervisor/port.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_mipi_dsi.h>

// Statically allocate the MIPI DSI bus (only one DSI bus on ESP32-P4)
static mipidsi_bus_obj_t board_mipidsi_bus;

// ST7701S initialization sequence for Tanmatsu display (480x800)
// Format: cmd, len|delay_flag, [data bytes], [delay_ms if delay_flag set]
// delay_flag = 0x80
static const uint8_t st7701s_init_sequence[] = {
    // Regular command function
    0xFF, 0x05, 0x77, 0x01, 0x00, 0x00, 0x00,
    // Turn on normal display mode (NORON)
    0x13, 0x00,
    // Undocumented
    0xEF, 0x01, 0x08,

    // Command 2 BK0 function
    0xFF, 0x05, 0x77, 0x01, 0x00, 0x00, 0x10,
    // LNESET (Display Line Setting): (0x63+1)*8 = 800 lines
    0xC0, 0x02, 0x63, 0x00,
    // PORCTRL (Porch Control): VBP = 16, VFP = 2
    0xC1, 0x02, 0x10, 0x02,
    // INVSET (Inversion sel. & frame rate control): PCLK=512+(8*16) = 640
    0xC2, 0x02, 0x37, 0x08,
    // Undocumented
    0xCC, 0x01, 0x38,
    // PVGAMCTRL (Positive Voltage Gamma Control)
    0xB0, 0x10, 0x40, 0xC9, 0x90, 0x0D, 0x0F, 0x04, 0x00, 0x07,
                0x07, 0x1C, 0x04, 0x52, 0x0F, 0xDF, 0x26, 0xCF,
    // NVGAMCTRL (Negative Voltage Gamma Control)
    0xB1, 0x10, 0x40, 0xC9, 0xCF, 0x0C, 0x90, 0x04, 0x00, 0x07,
                0x08, 0x1B, 0x06, 0x55, 0x13, 0x62, 0xE7, 0xCF,

    // Command 2 BK1 function
    0xFF, 0x05, 0x77, 0x01, 0x00, 0x00, 0x11,
    // VRHS
    0xB0, 0x01, 0x5D,
    // VCOMS
    0xB1, 0x01, 0x2D,
    // VGH
    0xB2, 0x01, 0x07,
    // TESTCMD
    0xB3, 0x01, 0x80,
    // VGLS
    0xB5, 0x01, 0x08,
    // PWCTRL1
    0xB7, 0x01, 0x85,
    // PWCTRL2
    0xB8, 0x01, 0x20,
    // DGMLUTR
    0xB9, 0x01, 0x10,
    // SPD1
    0xC1, 0x01, 0x78,
    // SPD2
    0xC2, 0x01, 0x78,
    // MIPISET1 (with 100ms delay)
    0xD0, 0x81, 0x88, 0x64,
    // Undocumented timing registers
    0xE0, 0x03, 0x00, 0x19, 0x02,
    0xE1, 0x0B, 0x05, 0xA0, 0x07, 0xA0, 0x04, 0xA0, 0x06, 0xA0,
               0x00, 0x44, 0x44,
    0xE2, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00,
    0xE3, 0x04, 0x00, 0x00, 0x33, 0x33,
    0xE4, 0x02, 0x44, 0x44,
    0xE5, 0x10, 0x0D, 0x31, 0xC8, 0xAF, 0x0F, 0x33, 0xC8, 0xAF,
               0x09, 0x2D, 0xC8, 0xAF, 0x0B, 0x2F, 0xC8, 0xAF,
    0xE6, 0x04, 0x00, 0x00, 0x33, 0x33,
    0xE7, 0x02, 0x44, 0x44,
    0xE8, 0x10, 0x0C, 0x30, 0xC8, 0xAF, 0x0E, 0x32, 0xC8, 0xAF,
               0x08, 0x2C, 0xC8, 0xAF, 0x0A, 0x2E, 0xC8, 0xAF,
    0xEB, 0x07, 0x02, 0x00, 0xE4, 0xE4, 0x44, 0x00, 0x40,
    0xEC, 0x02, 0x3C, 0x00,
    0xED, 0x10, 0xAB, 0x89, 0x76, 0x54, 0x01, 0xFF, 0xFF, 0xFF,
               0xFF, 0xFF, 0xFF, 0x10, 0x45, 0x67, 0x98, 0xBA,

    // Regular command function
    0xFF, 0x05, 0x77, 0x01, 0x00, 0x00, 0x00,
    // Exit sleep mode (with 120ms delay)
    0x11, 0x80, 0x78,
    // Enable Tearing Effect output (TEON) - helps prevent tearing
    0x35, 0x01, 0x00,
    // Display on
    0x29, 0x00,
};

// Display timing parameters from Tanmatsu driver
// FPS = 30000000/(40+40+30+480)/(16+16+2+800) = ~60Hz
#define ST7701S_WIDTH             480
#define ST7701S_HEIGHT            800
#define ST7701S_PIXEL_CLOCK_MHZ   30
#define ST7701S_HSYNC_PULSE_WIDTH 40
#define ST7701S_HSYNC_BACK_PORCH  40
#define ST7701S_HSYNC_FRONT_PORCH 30
#define ST7701S_VSYNC_PULSE_WIDTH 16
#define ST7701S_VSYNC_BACK_PORCH  16
#define ST7701S_VSYNC_FRONT_PORCH 2
#define ST7701S_LANE_BITRATE_MBPS 500

// Reset pin
#define LCD_RESET_PIN &pin_GPIO14

void board_init(void) {
    // Reset the display
    digitalio_digitalinout_obj_t reset_pin;
    reset_pin.base.type = &digitalio_digitalinout_type;
    common_hal_digitalio_digitalinout_construct(&reset_pin, LCD_RESET_PIN);
    common_hal_digitalio_digitalinout_switch_to_output(&reset_pin, false, DRIVE_MODE_PUSH_PULL);

    // Pulse reset low then high
    common_hal_digitalio_digitalinout_set_value(&reset_pin, false);
    mp_hal_delay_ms(10);
    common_hal_digitalio_digitalinout_set_value(&reset_pin, true);
    mp_hal_delay_ms(120);

    common_hal_digitalio_digitalinout_deinit(&reset_pin);

    // Initialize the statically allocated MIPI DSI bus
    // Bus frequency = lane_bitrate * num_lanes = 500 Mbps * 2 = 1000 MHz (but we pass lane rate)
    board_mipidsi_bus.base.type = &mipidsi_bus_type;
    common_hal_mipidsi_bus_construct(&board_mipidsi_bus, ST7701S_LANE_BITRATE_MBPS * 1000000, 2);  // 2 lanes

    // Allocate display bus for the display object
    primary_display_bus_t *display_bus_obj = allocate_display_bus_or_raise();
    mipidsi_display_obj_t *display = &display_bus_obj->mipidsi;
    display->base.type = &mipidsi_display_type;

    common_hal_mipidsi_display_construct(
        display,
        &board_mipidsi_bus,                         // Use statically allocated bus
        st7701s_init_sequence,
        sizeof(st7701s_init_sequence),
        0,                                          // virtual_channel
        ST7701S_WIDTH,                              // width
        ST7701S_HEIGHT,                             // height
        90,                                         // rotation
        16,                                         // color_depth (RGB565)
        NULL,                                       // backlight_pin (handled by coprocessor)
        1.0f,                                       // brightness
        60,                                         // native_frames_per_second
        true,                                       // backlight_on_high
        ST7701S_HSYNC_PULSE_WIDTH,
        ST7701S_HSYNC_BACK_PORCH,
        ST7701S_HSYNC_FRONT_PORCH,
        ST7701S_VSYNC_PULSE_WIDTH,
        ST7701S_VSYNC_BACK_PORCH,
        ST7701S_VSYNC_FRONT_PORCH,
        ST7701S_PIXEL_CLOCK_MHZ * 1000000           // pixel_clock_frequency in Hz
        );

    // Create framebuffer display
    framebufferio_framebufferdisplay_obj_t *fb_display = &allocate_display()->framebuffer_display;
    fb_display->base.type = &framebufferio_framebufferdisplay_type;

    common_hal_framebufferio_framebufferdisplay_construct(
        fb_display,
        MP_OBJ_FROM_PTR(display),
        90,                                         // rotation
        true                                        // auto_refresh
        );
}

// Virtual keyboard input injection for Tanmatsu keyboard
// Ring buffer for queued characters to inject into sys.stdin
static ringbuf_t tanmatsu_keyboard_queue;
static char tanmatsu_keyboard_buf[64];
static bool tanmatsu_keyboard_serial_attached = false;
static bool tanmatsu_keyboard_first_read = true;  // Track first read after attachment

// Python callback for keyboard polling
static mp_obj_t tanmatsu_keyboard_poll_callback = MP_OBJ_NULL;
static bool tanmatsu_keyboard_use_polling = false;
static uint64_t tanmatsu_keyboard_start_time = 0;
static bool tanmatsu_keyboard_started = false;

// Background callback to call Python polling function
// Keyboard input system inspired by Cardputer's keypad module approach
static background_callback_t tanmatsu_keyboard_poll_cb = {NULL, NULL, NULL, NULL};

static void tanmatsu_keyboard_poll_background(void *unused) {
    if (!tanmatsu_keyboard_use_polling || tanmatsu_keyboard_poll_callback == MP_OBJ_NULL) {
        return;
    }
    
    // Wait at least 1 second after attach_serial() before starting to call Python
    // This gives the system time to fully initialize
    if (!tanmatsu_keyboard_started) {
        uint64_t now = port_get_raw_ticks(NULL);
        if (tanmatsu_keyboard_start_time == 0) {
            tanmatsu_keyboard_start_time = now;
        }
        if (now - tanmatsu_keyboard_start_time < 1024) {  // ~1 second at 1024Hz
            // Re-add callback to check again later
            background_callback_add_core(&tanmatsu_keyboard_poll_cb);
            return;
        }
        tanmatsu_keyboard_started = true;
    }
    
    // Call Python callback function
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_call_function_0(tanmatsu_keyboard_poll_callback);
        nlr_pop();
    } else {
        // Exception occurred - disable polling to prevent crash loop
        tanmatsu_keyboard_use_polling = false;
        tanmatsu_keyboard_started = false;
    }
    
    // Re-add callback to continue polling
    if (tanmatsu_keyboard_use_polling && tanmatsu_keyboard_poll_callback != MP_OBJ_NULL) {
        background_callback_add_core(&tanmatsu_keyboard_poll_cb);
    }
}

// Override board_serial_init() - initialize ring buffer
void board_serial_init(void) {
    ringbuf_init(&tanmatsu_keyboard_queue, 
                 (uint8_t *)tanmatsu_keyboard_buf, 
                 sizeof(tanmatsu_keyboard_buf));
    
    // Set up polling callback
    tanmatsu_keyboard_poll_cb.fun = tanmatsu_keyboard_poll_background;
    tanmatsu_keyboard_poll_cb.data = NULL;
}

// Note: port_background_tick() is defined in ports/espressif/background.c
// We can't easily override it from board.c. Instead, we'll use the background
// callback system which is called from supervisor_background_tick().

// Override board_serial_connected() - return true when keyboard is attached
bool board_serial_connected(void) {
    return tanmatsu_keyboard_serial_attached;
}

// Override board_serial_bytes_available() - return count of queued characters
uint32_t board_serial_bytes_available(void) {
    if (tanmatsu_keyboard_serial_attached) {
        return ringbuf_num_filled(&tanmatsu_keyboard_queue);
    }
    return 0;
}

// Override board_serial_read() - inject characters into sys.stdin
char board_serial_read(void) {
    if (tanmatsu_keyboard_serial_attached && 
        ringbuf_num_filled(&tanmatsu_keyboard_queue) > 0) {
        int c = ringbuf_get(&tanmatsu_keyboard_queue);
        if (c == -1) {
            // Ring buffer returned -1 (empty), shouldn't happen but handle it
            return -1;
        }
        if (c == mp_interrupt_char) {
            ringbuf_clear(&tanmatsu_keyboard_queue);
            mp_sched_keyboard_interrupt();
            return -1;
        }
        
        // No special handling needed - Python duplicates first character
        
        return (char)c;
    }
    return -1;  // Let other serial sources be checked
}

// Python function: tanmatsu_keyboard.write_char(char)
static mp_obj_t tanmatsu_keyboard_write_char(mp_obj_t char_obj) {
    char c = (char)mp_obj_get_int(char_obj);
    if (ringbuf_num_empty(&tanmatsu_keyboard_queue) > 0) {
        ringbuf_put(&tanmatsu_keyboard_queue, c);
    }
    // If buffer is full, drop the character (could raise error instead)
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(tanmatsu_keyboard_write_char_obj, tanmatsu_keyboard_write_char);

// Python function: tanmatsu_keyboard.write_string(str)
static mp_obj_t tanmatsu_keyboard_write_string(mp_obj_t str_obj) {
    size_t len;
    const char *str = mp_obj_str_get_data(str_obj, &len);
    for (size_t i = 0; i < len; i++) {
        // Wait for space in buffer (with timeout to avoid infinite loop)
        int retries = 100;
        while (ringbuf_num_empty(&tanmatsu_keyboard_queue) == 0 && retries > 0) {
            mp_hal_delay_us(100);  // Wait 100 microseconds
            retries--;
        }
        if (ringbuf_num_empty(&tanmatsu_keyboard_queue) > 0) {
            ringbuf_put(&tanmatsu_keyboard_queue, str[i]);
        }
        // If buffer is still full after retries, drop the character
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(tanmatsu_keyboard_write_string_obj, tanmatsu_keyboard_write_string);

// Python function: tanmatsu_keyboard.attach_serial()
static mp_obj_t tanmatsu_keyboard_attach_serial(void) {
    tanmatsu_keyboard_serial_attached = true;
    tanmatsu_keyboard_first_read = true;  // Reset first read flag
    // Clear any stale data in buffer when attaching
    ringbuf_clear(&tanmatsu_keyboard_queue);
    
    // Reset start timing
    tanmatsu_keyboard_start_time = 0;
    tanmatsu_keyboard_started = false;
    
    // Start polling if callback is set
    // Note: We'll wait 1 second before actually calling the Python callback
    if (tanmatsu_keyboard_poll_callback != MP_OBJ_NULL && tanmatsu_keyboard_use_polling) {
        background_callback_add_core(&tanmatsu_keyboard_poll_cb);
    }
    
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(tanmatsu_keyboard_attach_serial_obj, tanmatsu_keyboard_attach_serial);

// Python function: tanmatsu_keyboard.detach_serial()
static mp_obj_t tanmatsu_keyboard_detach_serial(void) {
    tanmatsu_keyboard_serial_attached = false;
    tanmatsu_keyboard_use_polling = false;
    tanmatsu_keyboard_started = false;
    tanmatsu_keyboard_start_time = 0;
    
    ringbuf_clear(&tanmatsu_keyboard_queue);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(tanmatsu_keyboard_detach_serial_obj, tanmatsu_keyboard_detach_serial);

// Python function: tanmatsu_keyboard.set_poll_callback(callback, use_polling=True)
static mp_obj_t tanmatsu_keyboard_set_poll_callback(size_t n_args, const mp_obj_t *args) {
    tanmatsu_keyboard_poll_callback = args[0];
    
    if (n_args >= 2) {
        tanmatsu_keyboard_use_polling = mp_obj_is_true(args[1]);
    } else {
        tanmatsu_keyboard_use_polling = true;
    }
    
    // Start polling if attached and enabled
    if (tanmatsu_keyboard_serial_attached && tanmatsu_keyboard_use_polling && 
        tanmatsu_keyboard_poll_callback != MP_OBJ_NULL) {
        background_callback_add_core(&tanmatsu_keyboard_poll_cb);
    }
    
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tanmatsu_keyboard_set_poll_callback_obj, 1, 2, tanmatsu_keyboard_set_poll_callback);


// Register Python module
static const mp_rom_map_elem_t tanmatsu_keyboard_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_tanmatsu_keyboard) },
    { MP_ROM_QSTR(MP_QSTR_write_char), MP_ROM_PTR(&tanmatsu_keyboard_write_char_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_string), MP_ROM_PTR(&tanmatsu_keyboard_write_string_obj) },
    { MP_ROM_QSTR(MP_QSTR_attach_serial), MP_ROM_PTR(&tanmatsu_keyboard_attach_serial_obj) },
    { MP_ROM_QSTR(MP_QSTR_detach_serial), MP_ROM_PTR(&tanmatsu_keyboard_detach_serial_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_poll_callback), MP_ROM_PTR(&tanmatsu_keyboard_set_poll_callback_obj) },
};
static MP_DEFINE_CONST_DICT(tanmatsu_keyboard_module_globals, tanmatsu_keyboard_module_globals_table);

const mp_obj_module_t tanmatsu_keyboard_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&tanmatsu_keyboard_module_globals,
};
MP_REGISTER_MODULE(MP_QSTR_tanmatsu_keyboard, tanmatsu_keyboard_module);

// Use the MP_WEAK supervisor/shared/board.c versions of routines not defined here.
