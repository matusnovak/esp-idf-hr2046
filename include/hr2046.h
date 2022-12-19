#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>

#define HR2046_CH_X 0x10
#define HR2046_CH_Y 0x50
#define HR2046_CH_Z1 0x30
#define HR2046_CH_Z2 0x40

typedef struct hr2046_context_t* hr2046_handle_t;

struct hr2046_config_t {
    spi_host_device_t spi_host;
    int cs_pin;
    int irq_pin;
};

typedef struct hr2046_config_t hr2046_config_t;

esp_err_t hr2046_init(const hr2046_config_t* config, hr2046_handle_t* handle);
esp_err_t hr2046_send_cmd(hr2046_handle_t handle, uint8_t cmd);
esp_err_t hr2046_power_down(hr2046_handle_t handle);
esp_err_t hr2046_read(hr2046_handle_t handle, uint8_t channel, uint16_t* value);
esp_err_t hr2046_read_xyz(hr2046_handle_t handle, uint16_t* x, uint16_t* y, uint16_t* z);
esp_err_t hr2046_has_touch(hr2046_handle_t handle, int* touch);
esp_err_t hr2046_clear_touch(hr2046_handle_t handle);
