#include "sys_nibe.h"
#include "nibe_protocol.h"

#include <cstring>
#include <freertos/FreeRTOS.h>

#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace sys {

constexpr auto TAG = "sys.nibeclient";

constexpr int MAX_MSG_SIZE = 512;
constexpr int TIMEOUT_SYM = 2;


void log_bytes(const uint8_t* data, size_t len, const char* pre, const char* post)
{
    char buffer[256];
    for (size_t i = 0; i < len && i < 85; i++) {
        sprintf(&buffer[0] + i * 3, "%02X ", data[i]);
    }
    ESP_LOGI(TAG, "%s%s%s", pre, buffer, post);
}


void uart_task_wrapper(void* arg)
{
    nibeclient* instance = static_cast<nibeclient*>(arg);
    instance->uart_event_task();
    vTaskDelete(NULL);
}


void nibeclient::uart_event_task()
{
    uart_event_t event;
    uint8_t buffer[MAX_MSG_SIZE];

    while (true) {
        if (xQueueReceive(m_evt_queue, &event, portMAX_DELAY) == pdFALSE) {
            continue;
        }

        switch (event.type) {

        case UART_DATA: {
            int read = uart_read_bytes(m_config.uart_port, &buffer, event.size, portMAX_DELAY);
            if (read > 0) {
                run_parser(&buffer[0], read);
            }
        } break;
        case UART_BREAK: {
            ESP_LOGI(TAG, "UART_BREAK");
        } break;
        case UART_BUFFER_FULL: {
            ESP_LOGE(TAG, "UART_BUFFER_FULL");
            uart_flush_input(m_config.uart_port);
            xQueueReset(m_evt_queue);
        } break;
        case UART_FIFO_OVF: {
            ESP_LOGE(TAG, "UART_FIFO_OVF");
            uart_flush_input(m_config.uart_port);
            xQueueReset(m_evt_queue);
        } break;
        case UART_FRAME_ERR: {
            ESP_LOGE(TAG, "UART_FRAME_ERR");
        } break;
        case UART_PARITY_ERR: {
            ESP_LOGE(TAG, "UART_PARITY_ERR");
        } break;
        case UART_DATA_BREAK: {
            ESP_LOGI(TAG, "UART_DATA_BREAK");
        } break;
        default:
            break;
        }
    }
}


nibeclient::nibeclient(config_t cfg)
    : m_config(cfg)
    , m_state(parser_state::WAIT_MASTER_START)
    , m_req_index(0)
    , m_rsp_index(0)
{
}


esp_err_t nibeclient::start()
{
    uart_config_t uartconf = { .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT };

    // To be called first!
    esp_err_t ec = uart_driver_install(
        m_config.uart_port, MAX_MSG_SIZE * 2, MAX_MSG_SIZE * 2, 10, &m_evt_queue, 0);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install(): %s", esp_err_to_name(ec));
        return ec;
    }

    ec = uart_param_config(m_config.uart_port, &uartconf);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config(): %s", esp_err_to_name(ec));
        return ec;
    }

    ec = uart_set_pin(m_config.uart_port, m_config.uart_tx_pin, m_config.uart_rx_pin,
        m_config.enable_pin, UART_PIN_NO_CHANGE);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin(): %s", esp_err_to_name(ec));
        return ec;
    }

    // Driver manages DE/RE of the transceiver via the RTS pin.
    if (m_config.enable_pin > GPIO_NUM_NC) {
        ec = uart_set_mode(m_config.uart_port, UART_MODE_RS485_HALF_DUPLEX);
        if (ec != ESP_OK) {
            ESP_LOGE(TAG, "uart_set_mode(): %s", esp_err_to_name(ec));
            return ec;
        }
    }

    // Have an interrupt fired `TIMEOUT_SYM` symbols after a transmission.
    ec = uart_set_rx_timeout(m_config.uart_port, TIMEOUT_SYM);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_rx_timeout(): %s", esp_err_to_name(ec));
        return ec;
    }

    xTaskCreatePinnedToCore(&uart_task_wrapper, "uart_events", configMINIMAL_STACK_SIZE * 2, this,
        tskIDLE_PRIORITY + 1, &m_evt_task, 1);

    return ESP_OK;
}


void nibeclient::run_parser(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        uint8_t byte = data[i];
        switch (m_state) {
        case parser_state::WAIT_MASTER_START: {
            m_req_index = 0;
            if (byte == nibe_protocol::control_bytes::MASTER_START) {
                m_req_msg[m_req_index++] = byte;
                m_state = parser_state::WAIT_MASTER_BODY;
            } else {
                ESP_LOGI(TAG, "? > ?: %02X - not a frame start.", byte);
            }
        } break;
        case parser_state::WAIT_MASTER_BODY: {
            m_req_msg[m_req_index++] = byte;

            // Await header
            if (m_req_index < nibe_protocol::MASTER_HDR_LEN) {
                continue;
            }

            // Await payload + checksum
            uint8_t len = m_req_msg[4];
            if (m_req_index < nibe_protocol::MASTER_HDR_LEN + len + 1) {
                continue;
            }

            m_state
                = master_message_received(&m_req_msg[0], nibe_protocol::MASTER_HDR_LEN + len + 1);
        } break;
        case parser_state::WAIT_SLAVE_START: {
            m_rsp_index = 0;
            if (byte == nibe_protocol::control_bytes::ACK
                || byte == nibe_protocol::control_bytes::NACK) {
                m_rsp_msg[m_rsp_index++] = byte;
                m_state = parser_state::WAIT_MASTER_START;
                ESP_LOGI(TAG, "S > M: %02X - expected N/ACK.", byte);
                // Notify message complete
                continue;
            }

            if (byte == nibe_protocol::control_bytes::SLAVE_START) {
                m_rsp_msg[m_rsp_index++] = byte;
                m_state = parser_state::WAIT_SLAVE_BODY;
                continue;
            }

            ESP_LOGI(TAG, "S > M: %02X - unexpected response.", byte);
            m_state = parser_state::WAIT_MASTER_START;
        } break;
        case parser_state::WAIT_SLAVE_BODY: {
            m_rsp_msg[m_rsp_index++] = byte;

            // Await header
            if (m_rsp_index < nibe_protocol::SLAVE_HDR_LEN) {
                continue;
            }

            // Await payload + checksum
            uint8_t len = m_rsp_msg[2];
            if (m_rsp_index < nibe_protocol::SLAVE_HDR_LEN + len + 1) {
                continue;
            }

            m_state = slave_message_received(&m_rsp_msg[0], nibe_protocol::SLAVE_HDR_LEN + len + 1);
        } break;
        case parser_state::WAIT_MASTER_ACK: {
            if (byte != nibe_protocol::control_bytes::ACK
                && byte != nibe_protocol::control_bytes::NACK) {
                ESP_LOGI(TAG, "M > S: %02X - unexpected N/ACK.", byte);
            } else {
                ESP_LOGI(TAG, "M > S: %02X - expected N/ACK.", byte);
            }

            m_state = parser_state::WAIT_MASTER_START;
        } break;
        }
    }
}


nibeclient::parser_state nibeclient::master_message_received(const uint8_t* message, size_t size)
{
    uint16_t addr = (message[1] << 8) | (message[2]);
    bool for_me = addr == m_config.address;

    uint8_t xorsum = message[size - 1];
    uint8_t expected = nibe_protocol::xorsum(&message[1], size - 2);
    if (xorsum != expected) {
        log_bytes(message, size, "M > S: ", " - bad checksum.");
    } else {
        log_bytes(message, size, "M > S: ", " - message is valid.");
    }

    if (for_me) {
        auto resp = handle_message(message, size);
        if (resp.size() > 0) {
            uart_write_bytes(m_config.uart_port, resp.data(), resp.size());
            log_bytes(resp.data(), resp.size(), "S > M: ", " - our response.");
        }
        // If we send ACK/NACK, the sequence is complete. Otherwise master ACKs.
        return resp.size() <= 1 ? parser_state::WAIT_MASTER_START : parser_state::WAIT_MASTER_ACK;
    } else {
        // Slave will report ACK/NACK/own data.
        return parser_state::WAIT_SLAVE_START;
    }
}


nibeclient::parser_state nibeclient::slave_message_received(const uint8_t* message, size_t size)
{
    uint8_t xorsum = message[size - 1];
    uint8_t expected = nibe_protocol::xorsum(&message[0], size - 1);
    if (xorsum != expected) {
        log_bytes(message, size, "S > M: ", " - bad checksum.");
        return parser_state::WAIT_MASTER_ACK;
    }

    log_bytes(message, size, "S > M: ", " - message is valid.");
    return parser_state::WAIT_MASTER_ACK;
}


int dedup_copy(uint8_t* dst, const uint8_t* src, size_t size)
{
    if (size == 0)
        return 0;

    const uint8_t* ori = dst;
    const uint8_t* end = src + size - 1;
    while (src < end) {
        if ((*src) == 0x5C && *(src + 1) == 0x5C)
            src++;
        *dst++ = *src++;
    }

    *dst++ = *src++;
    return dst - ori - 1;
}


std::vector<uint8_t> nibeclient::handle_message(const uint8_t* message, size_t size)
{
    using namespace nibe_protocol;

    std::vector<uint8_t> resp;

    auto cmd = static_cast<command_bytes>(message[MASTER_FRAME_POS_CMD]);

    size_t orig_datalen = message[MASTER_FRAME_POS_LEN];
    const uint8_t* orig_data = message + MASTER_FRAME_POS_DATA;

    // The heatpump escapes 0x5C in payload by sending 0x5C, 0x5C. Probably to help
    // parsers to avoid false starts. We have to first deduplicate the data before
    // evaluating it. Sending data with 0x5C seems unproblematic.
    size_t datalen = dedup_copy(m_scratch, orig_data, orig_datalen);
    const uint8_t* data = m_scratch;

    switch (cmd) {

    case nibe_protocol::MODBUS_DATA_MSG: {
        size_t nelem = datalen / 4;
        std::vector<datapoint> fields;
        fields.reserve(nelem);

        for (int i = 0; i < nelem; ++i) {
            datapoint f;
            f.reg = data[0] + (data[1] << 8);
            f.value = data[2] + (data[3] << 8);
            data += 4;

            fields.push_back(f);
        }

        m_config.on_periodic_update(fields);
        resp.push_back(control_bytes::ACK);
    } break;
    case nibe_protocol::MODBUS_READ_REQ: {
        uint16_t reg_to_read = 0;
        if (m_config.on_read_request(reg_to_read)) {
            resp.push_back(control_bytes::SLAVE_START);
            resp.push_back(command_bytes::MODBUS_READ_REQ);
            resp.push_back(sizeof(uint16_t));
            resp.push_back(static_cast<uint8_t>((reg_to_read & 0xFF)));
            resp.push_back(static_cast<uint8_t>((reg_to_read >> 8) & 0xFF));
            resp.push_back(xorsum(resp.data(), resp.size()));
        } else {
            resp.push_back(control_bytes::ACK);
        }
    } break;
    case nibe_protocol::MODBUS_READ_RESP: {
        if (datalen == 6) {
            datapoint p;
            p.reg = data[0] + (data[1] << 8);
            p.value = data[2] + (data[3] << 8) + (data[4] << 16) + (data[5] << 24);
            m_config.on_read_response(p);
            resp.push_back(control_bytes::ACK);
        } else {
            resp.push_back(control_bytes::NACK);
        }
    } break;
    case nibe_protocol::MODBUS_WRITE_REQ: {
        datapoint field_to_write;
        if (m_config.on_write_request(field_to_write)) {
            resp.push_back(control_bytes::SLAVE_START);
            resp.push_back(command_bytes::MODBUS_WRITE_REQ);
            resp.push_back(6);
            resp.push_back(static_cast<uint8_t>((field_to_write.reg & 0xFF)));
            resp.push_back(static_cast<uint8_t>((field_to_write.reg >> 8) & 0xFF));
            resp.push_back(static_cast<uint8_t>((field_to_write.value & 0xFF)));
            resp.push_back(static_cast<uint8_t>((field_to_write.value >> 8) & 0xFF));
            resp.push_back(static_cast<uint8_t>((field_to_write.value >> 16) & 0xFF));
            resp.push_back(static_cast<uint8_t>((field_to_write.value >> 24) & 0xFF));
            resp.push_back(xorsum(resp.data(), resp.size()));
        } else {
            resp.push_back(control_bytes::ACK);
        }
    } break;
    case nibe_protocol::MODBUS_WRITE_RESP: {
        if (datalen == 1) {
            bool success = data[0];
            m_config.on_write_response(0xFFFF, success);
            resp.push_back(control_bytes::ACK);
        } else {
            resp.push_back(control_bytes::NACK);
        }
    } break;
    case nibe_protocol::ACCESSORY_INFO_REQ: {
        resp.push_back(control_bytes::SLAVE_START);
        resp.push_back(command_bytes::ACCESSORY_INFO_REQ);
        resp.push_back(3);
        resp.push_back(0x0A); // Version LSB
        resp.push_back(0x00); // Version MSB
        resp.push_back(0x01); // Modbus Address?
        resp.push_back(xorsum(resp.data(), resp.size()));
    } break;
    default:
        break;
    }

    return resp;
}

}