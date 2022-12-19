#include "hr2046.h"

#include <unistd.h>

// https://github.com/giovannitarter/Arduino/blob/21eb2c47e09fa6421335eeb04299c6d5ff8b60ea/touch_and_temp/XPT2046_Touchscreen.cpp

#define CMD_START 0x80

#define CMD_12BIT 0x00
#define CMD_8BIT 0x08

#define CMD_DFR 0x00
#define CMD_SER 0x04

#define CMD_REF_ON 0x02
#define CMD_REF_OFF 0x00

#define CMD_ADC_ON 0x01
#define CMD_ADC_OFF 0x00

/*#define ADS_SER (1 << 2)

#define CMD_VAUX (0x6 << 4)
#define CMD_POS_X (0x5 << 4)
#define CMD_POS_Y (0x1 << 4)
#define CMD_POS_Z1 (0x3 << 4)
#define CMD_POS_Z2 (0x4 << 4)

#define CMD_ADC_ON 0x1
#define CMD_REF_ON 0x2

#define REF_ON (CMD_START | CMD_12BIT | CMD_DFR | CMD_POS_X | CMD_ADC_ON | CMD_REF_ON)
#define PWRDOWN (CMD_START | CMD_12BIT | CMD_DFR | CMD_POS_X)

#define READ_X 0xD0
#define READ_Y 0x90*/

struct hr2046_context_t {
    spi_device_handle_t spi;
    int cs_pin;
    int irq_pin;
    int touched;
};

typedef struct hr2046_context_t hr2046_context_t;

void hr2046_irq_handler(void* arg) {
    hr2046_handle_t handle = (hr2046_handle_t)(arg);
    handle->touched = 1;
}

esp_err_t hr2046_init(const hr2046_config_t* config, hr2046_handle_t* handle) {
    hr2046_context_t* ctx = (hr2046_context_t*)malloc(sizeof(hr2046_context_t));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 1 * 1000 * 1000;
    devcfg.spics_io_num = config->cs_pin;
    devcfg.queue_size = 2;
    devcfg.command_bits = 8;

    esp_err_t ret = spi_bus_add_device(config->spi_host, &devcfg, &ctx->spi);
    if (ret != ESP_OK) {
        free(ctx);
        return ret;
    }

    gpio_config_t io_conf = {};
    //io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = config->irq_pin;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        free(ctx);
        return ret;
    }

    ctx->touched = 0;

    /*ret = gpio_isr_handler_add(config->irq_pin, hr2046_irq_handler, (void*)ctx);
    if (ret != ESP_OK) {
        free(ctx);
        return ret;
    }*/

    *handle = ctx;

    return ESP_OK;
}

esp_err_t hr2046_send_cmd(hr2046_handle_t handle, uint8_t cmd) {
    if (!handle) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t txd = {};
    txd.rx_buffer = NULL;
    txd.tx_buffer = NULL;
    txd.length = 0;
    txd.rxlength = 0;
    txd.cmd = cmd;
    
    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, &txd);

    spi_device_release_bus(handle->spi);

    return ESP_OK;
}

esp_err_t hr2046_read(hr2046_handle_t handle, uint8_t channel, uint16_t* value) {
    if (!handle) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t txd = {};
    txd.rx_buffer = value;
    txd.tx_buffer = NULL;
    txd.length = 2 * 8;
    txd.rxlength = 2 * 8;
    txd.cmd = CMD_START | CMD_12BIT | CMD_ADC_ON | channel;
    
    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, &txd);

    spi_device_release_bus(handle->spi);

    return ESP_OK;
}

esp_err_t hr2046_read_xyz(hr2046_handle_t handle, uint16_t* x, uint16_t* y, uint16_t* z) {
    uint16_t z1;
    uint16_t z2;

    esp_err_t ret = hr2046_read(handle, HR2046_CH_X, x); // Dummy
    if (ret != ESP_OK) {
        return ret;
    }

    ret = hr2046_read(handle, HR2046_CH_X, x);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = hr2046_read(handle, HR2046_CH_Y, y);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = hr2046_read(handle, HR2046_CH_Z1, &z1);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = hr2046_read(handle, HR2046_CH_Z2, &z2);
    if (ret != ESP_OK) {
        return ret;
    }

    *z = (z1 + 4095) - z2;

    return ESP_OK;
}

esp_err_t hr2046_power_down(hr2046_handle_t handle) {
    return hr2046_send_cmd(handle, CMD_START | CMD_REF_OFF | CMD_ADC_OFF);
}

esp_err_t hr2046_has_touch(hr2046_handle_t handle, int* touch) {
    if (!handle) {
        return ESP_ERR_INVALID_STATE;
    }

    *touch = gpio_get_level(handle->irq_pin) == 0;

    // *touch = handle->touched;
    return ESP_OK;
}

esp_err_t hr2046_clear_touch(hr2046_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_STATE;
    }

    handle->touched = 0;
    return ESP_OK;
}
