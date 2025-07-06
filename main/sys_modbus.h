#pragma once

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_netif.h>

namespace sys {

class modbus {
public:
    struct config_t {
        esp_netif_t* netif;
    };

    struct read_request {
        uint16_t reg;
    };

    struct write_request {
        uint16_t reg;
        uint32_t value;
    };

    modbus(config_t cfg);
    esp_err_t start();

    QueueHandle_t get_read_queue() const;
    QueueHandle_t get_write_queue() const;
    void update_register_u16(uint16_t reg, uint16_t value);
    void update_register_u32(uint16_t reg, uint32_t value);

private:
    config_t m_config;
    void* m_slave;
};

}