// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/mipidsi/Display.h"
#include "shared-bindings/mipidsi/Bus.h"
#include "shared-bindings/pwmio/PWMOut.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/time/__init__.h"
#include "bindings/espidf/__init__.h"
#include <esp_lcd_panel_ops.h>
#include <esp_heap_caps.h>
#include "py/runtime.h"

#include "soc/soc_caps.h"
#if SOC_PPA_SUPPORTED
#if __has_include("driver/ppa.h")
#include "driver/ppa.h"
#else
#undef SOC_PPA_SUPPORTED
#endif
#endif

// Cache write-back function (should be from rom/cache.h but it's not always available)
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

void common_hal_mipidsi_display_construct(mipidsi_display_obj_t *self,
    mipidsi_bus_obj_t *bus,
    const uint8_t *init_sequence,
    size_t init_sequence_len,
    mp_uint_t virtual_channel,
    mp_uint_t width,
    mp_uint_t height,
    mp_int_t rotation,
    mp_uint_t color_depth,
    const mcu_pin_obj_t *backlight_pin,
    mp_float_t brightness,
    mp_uint_t native_frames_per_second,
    bool backlight_on_high,
    mp_uint_t hsync_pulse_width,
    mp_uint_t hsync_back_porch,
    mp_uint_t hsync_front_porch,
    mp_uint_t vsync_pulse_width,
    mp_uint_t vsync_back_porch,
    mp_uint_t vsync_front_porch,
    mp_uint_t pixel_clock_frequency) {
    self->bus = bus;
    self->virtual_channel = virtual_channel;
    
    // Initialize standard fields
    self->rotation = rotation;
    self->color_depth = color_depth;
    self->native_frames_per_second = native_frames_per_second;
    self->backlight_on_high = backlight_on_high;
    self->framebuffer = NULL;
    self->physical_framebuffer = NULL;
    self->ppa_handle = NULL;
    self->dbi_io_handle = NULL;
    self->dpi_panel_handle = NULL;

    // Create the DBI interface for sending commands
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = virtual_channel,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };

    CHECK_ESP_RESULT(esp_lcd_new_panel_io_dbi(bus->bus_handle, &dbi_config, &self->dbi_io_handle));

    // Determine the pixel format based on color depth
    lcd_color_format_t color_format;
    if (color_depth == 16) {
        color_format = LCD_COLOR_FMT_RGB565;
    } else if (color_depth == 24) {
        color_format = LCD_COLOR_FMT_RGB888;
    } else {
        common_hal_mipidsi_display_deinit(self);
        mp_raise_ValueError_varg(MP_ERROR_TEXT("Invalid %q"), MP_QSTR_color_depth);
    }

    // Create the DPI panel for sending pixel data
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = virtual_channel,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = pixel_clock_frequency / 1000000,
        .in_color_format = color_format,
        .num_fbs = 1,
        .video_timing = {
            .h_size = width,
            .v_size = height,
            .hsync_pulse_width = hsync_pulse_width,
            .hsync_back_porch = hsync_back_porch,
            .hsync_front_porch = hsync_front_porch,
            .vsync_pulse_width = vsync_pulse_width,
            .vsync_back_porch = vsync_back_porch,
            .vsync_front_porch = vsync_front_porch,
        },
        .flags = {
            .use_dma2d = false, // We use PPA manually if available
            .disable_lp = false,
        },
    };

    esp_err_t ret = esp_lcd_new_panel_dpi(bus->bus_handle, &dpi_config, &self->dpi_panel_handle);
    if (ret != ESP_OK) {
        common_hal_mipidsi_display_deinit(self);
        CHECK_ESP_RESULT(ret);
    }

    // Get the physical framebuffer allocated by the driver
    void *fb = NULL;
    ret = esp_lcd_dpi_panel_get_frame_buffer(self->dpi_panel_handle, 1, &fb);
    if (ret != ESP_OK || fb == NULL) {
        common_hal_mipidsi_display_deinit(self);
        CHECK_ESP_RESULT(ret);
    }

    // Check if we can use PPA for hardware rotation
    bool use_ppa = false;
    #if SOC_PPA_SUPPORTED
    if (rotation == 90 || rotation == 270) {
        ppa_client_config_t ppa_config = {
            .oper_type = PPA_OPERATION_SRM,
        };
        // Use a pointer cast to avoid strict strictness if ppa_client_handle_t is different
        // but it should be compatible with void*
        if (ppa_register_client(&ppa_config, (ppa_client_handle_t*)&self->ppa_handle) == ESP_OK) {
            
            size_t fb_size = width * height * (color_depth / 8);
            // Allocate a secondary logical framebuffer in PSRAM
            void *logical_fb = heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
            
            if (logical_fb) {
                use_ppa = true;
                self->physical_framebuffer = (uint8_t *)fb;
                self->framebuffer = (uint8_t *)logical_fb;
                self->framebuffer_size = fb_size;
                
                // Swap width and height for the logical view
                self->width = height;
                self->height = width;
                
                // Set rotation to 0 for displayio, so it renders linearly to our logical buffer
                // The rotation will happen in hardware during refresh
                // self->rotation remains the requested rotation (90/270) for our internal logic
            } else {
                // Failed to allocate logical framebuffer, fallback to software rotation
                ppa_unregister_client(self->ppa_handle);
                self->ppa_handle = NULL;
            }
        }
    }
    #endif

    if (!use_ppa) {
        self->width = width;
        self->height = height;
        self->framebuffer = (uint8_t *)fb;
        self->framebuffer_size = width * height * (color_depth / 8);
        self->physical_framebuffer = NULL; // Flag that we are using the physical FB directly
    }

    // Send initialization sequence
    #define DELAY 0x80
    uint32_t i = 0;
    while (i < init_sequence_len) {
        const uint8_t *cmd = init_sequence + i;
        uint8_t data_size = *(cmd + 1);
        bool delay = (data_size & DELAY) != 0;
        data_size &= ~DELAY;
        const uint8_t *data = cmd + 2;
        esp_lcd_panel_io_tx_param(self->dbi_io_handle, cmd[0], data, data_size);

        uint16_t delay_length_ms = 0;
        if (delay) {
            data_size++;
            delay_length_ms = *(cmd + 1 + data_size);
            if (delay_length_ms == 255) {
                delay_length_ms = 500;
            }
        }
        common_hal_time_delay_ms(delay_length_ms);
        i += 2 + data_size;
    }

    // Initialize the panel after sending init commands
    ret = esp_lcd_panel_init(self->dpi_panel_handle);
    if (ret != ESP_OK) {
        common_hal_mipidsi_display_deinit(self);
        CHECK_ESP_RESULT(ret);
    }

    // Setup backlight PWM if pin is provided
    self->backlight_inout.base.type = &mp_type_NoneType;
    if (backlight_pin != NULL && common_hal_mcu_pin_is_free(backlight_pin)) {
        #if (CIRCUITPY_PWMIO)
        pwmout_result_t result = common_hal_pwmio_pwmout_construct(&self->backlight_pwm, backlight_pin, 0, 50000, false);
        if (result != PWMOUT_OK) {
            self->backlight_inout.base.type = &digitalio_digitalinout_type;
            common_hal_digitalio_digitalinout_construct(&self->backlight_inout, backlight_pin);
            common_hal_never_reset_pin(backlight_pin);
        } else {
            self->backlight_pwm.base.type = &pwmio_pwmout_type;
            common_hal_pwmio_pwmout_never_reset(&self->backlight_pwm);
        }
        #else
        self->backlight_inout.base.type = &digitalio_digitalinout_type;
        common_hal_digitalio_digitalinout_construct(&self->backlight_inout, backlight_pin);
        common_hal_never_reset_pin(backlight_pin);
        #endif

        // Set initial brightness
        #if (CIRCUITPY_PWMIO)
        if (self->backlight_pwm.base.type == &pwmio_pwmout_type) {
            common_hal_pwmio_pwmout_set_duty_cycle(&self->backlight_pwm, (uint16_t)(brightness * 0xFFFF));
        } else
        #endif
        if (self->backlight_inout.base.type == &digitalio_digitalinout_type) {
            bool on = brightness > 0;
            if (!backlight_on_high) {
                on = !on;
            }
            common_hal_digitalio_digitalinout_set_value(&self->backlight_inout, on);
        }
    }
    mipidsi_bus_increment_use_count(self->bus);
}

void common_hal_mipidsi_display_deinit(mipidsi_display_obj_t *self) {
    if (common_hal_mipidsi_display_deinited(self)) {
        return;
    }

    // Cleanup backlight
    #if (CIRCUITPY_PWMIO)
    if (self->backlight_pwm.base.type == &pwmio_pwmout_type) {
        common_hal_pwmio_pwmout_deinit(&self->backlight_pwm);
    } else
    #endif
    if (self->backlight_inout.base.type == &digitalio_digitalinout_type) {
        common_hal_digitalio_digitalinout_deinit(&self->backlight_inout);
    }

    // Cleanup PPA and logical buffer
    #if SOC_PPA_SUPPORTED
    if (self->ppa_handle) {
        ppa_unregister_client(self->ppa_handle);
        self->ppa_handle = NULL;
    }
    #endif
    
    if (self->physical_framebuffer != NULL && self->framebuffer != NULL) {
        // We allocated a separate logical framebuffer
        heap_caps_free(self->framebuffer);
        self->framebuffer = NULL;
    }
    self->physical_framebuffer = NULL;

    // Delete the DPI panel
    if (self->dpi_panel_handle != NULL) {
        esp_lcd_panel_del(self->dpi_panel_handle);
        self->dpi_panel_handle = NULL;
    }

    // Delete the DBI interface
    if (self->dbi_io_handle != NULL) {
        esp_lcd_panel_io_del(self->dbi_io_handle);
        self->dbi_io_handle = NULL;
    }

    mipidsi_bus_decrement_use_count(self->bus);
    self->bus = NULL;
    self->framebuffer = NULL;
}

bool common_hal_mipidsi_display_deinited(mipidsi_display_obj_t *self) {
    return self->dpi_panel_handle == NULL;
}

void common_hal_mipidsi_display_refresh(mipidsi_display_obj_t *self) {
    #if SOC_PPA_SUPPORTED
    if (self->ppa_handle) {
        // PPA Rotation BLIT: Logical FB -> Physical FB
        
        uint32_t logical_w = self->width;
        uint32_t logical_h = self->height;
        uint32_t physical_w = self->height; // Swapped
        uint32_t physical_h = self->width;
        
        ppa_srm_rotation_angle_t angle = PPA_SRM_ROTATION_ANGLE_0;
        if (self->rotation == 90) {
            angle = PPA_SRM_ROTATION_ANGLE_90;
        } else if (self->rotation == 270) {
            angle = PPA_SRM_ROTATION_ANGLE_270;
        }

        ppa_pixel_format_t ppa_fmt = PPA_PIXEL_FORMAT_RGB565;
        if (self->color_depth == 24) {
            ppa_fmt = PPA_PIXEL_FORMAT_RGB888;
        }

        ppa_srm_oper_config_t srm_config = {
            .in.buffer = self->framebuffer,
            .in.pic_w = logical_w,
            .in.pic_h = logical_h,
            .in.block_w = logical_w,
            .in.block_h = logical_h,
            .in.pixel_format = ppa_fmt,
            .out.buffer = self->physical_framebuffer,
            .out.pic_w = physical_w,
            .out.pic_h = physical_h,
            .out.block_w = physical_w,
            .out.block_h = physical_h,
            .out.pixel_format = ppa_fmt,
            .rotation_angle = angle,
            .scale_x = 1.0f,
            .scale_y = 1.0f,
        };
        
        // This is a blocking call that waits for the PPA operation to complete
        ppa_do_srm(self->ppa_handle, &srm_config);

        // Flush the physical framebuffer cache so the LCD DMA sees the new data
        Cache_WriteBack_Addr((uint32_t)self->physical_framebuffer, self->framebuffer_size);
        
        // Notify panel (mostly valid for ensuring sync)
        esp_lcd_panel_draw_bitmap(self->dpi_panel_handle, 0, 0, physical_w, physical_h, self->physical_framebuffer);
        return;
    }
    #endif

    // Default behavior (Software Rotation or No Rotation)
    // Drawing the framebuffer we got from the IDF will flush the cache(s) so
    // DMA can see our changes. It won't cause an extra copy.
    esp_lcd_panel_draw_bitmap(self->dpi_panel_handle, 0, 0, self->width, self->height, self->framebuffer);
}

mp_float_t common_hal_mipidsi_display_get_brightness(mipidsi_display_obj_t *self) {
    return self->current_brightness;
}

bool common_hal_mipidsi_display_set_brightness(mipidsi_display_obj_t *self, mp_float_t brightness) {
    if (!self->backlight_on_high) {
        brightness = 1.0 - brightness;
    }
    bool ok = false;

    // Avoid PWM types and functions when the module isn't enabled
    #if (CIRCUITPY_PWMIO)
    bool ispwm = (self->backlight_pwm.base.type == &pwmio_pwmout_type) ? true : false;
    #else
    bool ispwm = false;
    #endif

    if (ispwm) {
        #if (CIRCUITPY_PWMIO)
        common_hal_pwmio_pwmout_set_duty_cycle(&self->backlight_pwm, (uint16_t)(0xffff * brightness));
        ok = true;
        #else
        ok = false;
        #endif
    } else if (self->backlight_inout.base.type == &digitalio_digitalinout_type) {
        common_hal_digitalio_digitalinout_set_value(&self->backlight_inout, brightness > 0.99);
        ok = true;
    }
    if (ok) {
        self->current_brightness = brightness;
    }
    return ok;
}

int common_hal_mipidsi_display_get_rotation(mipidsi_display_obj_t *self) {
    if (self->ppa_handle) {
        return 0; // If PPA is handling rotation, report 0 to displayio so it doesn't rotate again
    }
    return self->rotation;
}

void common_hal_mipidsi_display_set_rotation(mipidsi_display_obj_t *self, int rotation) {
    self->rotation = rotation;
}

int common_hal_mipidsi_display_get_width(mipidsi_display_obj_t *self) {
    return self->width;
}

int common_hal_mipidsi_display_get_height(mipidsi_display_obj_t *self) {
    return self->height;
}

int common_hal_mipidsi_display_get_row_stride(mipidsi_display_obj_t *self) {
    return self->width * (self->color_depth / 8);
}

int common_hal_mipidsi_display_get_color_depth(mipidsi_display_obj_t *self) {
    return self->color_depth;
}

int common_hal_mipidsi_display_get_native_frames_per_second(mipidsi_display_obj_t *self) {
    return self->native_frames_per_second;
}

bool common_hal_mipidsi_display_get_grayscale(mipidsi_display_obj_t *self) {
    return false;
}

mp_int_t common_hal_mipidsi_display_get_buffer(mp_obj_t self_in, mp_buffer_info_t *bufinfo, mp_uint_t flags) {
    mipidsi_display_obj_t *self = (mipidsi_display_obj_t *)self_in;

    bufinfo->buf = self->framebuffer;
    bufinfo->len = self->framebuffer_size;
    bufinfo->typecode = 'B';

    return 0;
}
