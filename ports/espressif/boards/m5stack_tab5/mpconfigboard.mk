USB_VID = 0x303A
USB_PID = 0x832B
USB_PRODUCT = "M5Stack Tab5"
USB_MANUFACTURER = "M5Stack"

IDF_TARGET = esp32p4

CIRCUITPY_ESP_FLASH_SIZE = 16MB
CIRCUITPY_ESP_FLASH_MODE = qio
CIRCUITPY_ESP_FLASH_FREQ = 80m

CIRCUITPY_ESP_PSRAM_SIZE = 32MB
CIRCUITPY_ESP_PSRAM_MODE = hpi
CIRCUITPY_ESP_PSRAM_FREQ = 200m

# ESP-Hosted co-processor support (ESP32-C6 connected via SDIO)
# This enables the SDIO transport layer infrastructure for the ESP32-C6 co-processor.
# Full WiFi support requires integrating the esp_hosted component from Espressif.
# See: https://github.com/espressif/esp-hosted
CIRCUITPY_ESP_HOSTED = 1

# TODO: Enable these once esp_hosted component is integrated:
# CIRCUITPY_WIFI = 1
# CIRCUITPY_SOCKETPOOL = 1
# CIRCUITPY_SSL = 1
# CIRCUITPY_MDNS = 1
# CIRCUITPY_HASHLIB = 1
