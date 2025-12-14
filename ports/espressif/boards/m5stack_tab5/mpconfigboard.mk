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
# The esp-hosted-mcu and esp-wifi-remote components provide WiFi via the co-processor
# See: https://github.com/espressif/esp-hosted-mcu
CIRCUITPY_ESP_HOSTED = 1

# WiFi and networking enabled via ESP-Hosted
CIRCUITPY_WIFI = 1
CIRCUITPY_SOCKETPOOL = 1
CIRCUITPY_SSL = 1
CIRCUITPY_MDNS = 1
CIRCUITPY_HASHLIB = 1
