#pragma once

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>

#include <cstddef>
#include <cstdint>
#include <functional>


namespace sys {

extern "C" void uart_task_wrapper(void*);

class nibeclient {
public:
    struct datapoint {
        uint16_t reg;
        uint32_t value;
    };

    struct config_t {
        // hardware
        bool enable_necessary;
        gpio_num_t enable_pin;
        uart_port_t uart_port;
        gpio_num_t uart_tx_pin;
        gpio_num_t uart_rx_pin;
        // protocol
        uint16_t address;

        std::function<bool(uint16_t& req)> on_read_request;
        std::function<void(datapoint resp)> on_read_response;

        std::function<bool(datapoint& req)> on_write_request;
        std::function<void(uint16_t resp, bool success)> on_write_response;
        std::function<bool(const std::vector<datapoint>&)> on_periodic_update;
    };

    nibeclient(config_t cfg);
    esp_err_t start();

private:
    enum class parser_state {
        WAIT_MASTER_START,
        WAIT_MASTER_BODY,
        WAIT_SLAVE_START,
        WAIT_SLAVE_BODY,
        WAIT_MASTER_ACK
    };

    friend void uart_task_wrapper(void*);
    void uart_event_task();

    void run_parser(uint8_t* data, size_t size);
    parser_state master_message_received(const uint8_t* message, size_t size);
    parser_state slave_message_received(const uint8_t* message, size_t size);

    std::vector<uint8_t> handle_message(const uint8_t* message, size_t size);

    /* Scaffolding */
    config_t m_config;
    QueueHandle_t m_evt_queue;
    TaskHandle_t m_evt_task;

    /* Parsing */
    parser_state m_state;
    size_t m_req_index;
    uint8_t m_req_msg[128];
    size_t m_rsp_index;
    uint8_t m_rsp_msg[128];
    uint8_t m_scratch[128];
};

}