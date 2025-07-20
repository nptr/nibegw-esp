#include "sdkconfig.h"
#include "sys_modbus.h"
#include "sys_nibe.h"
#include "sys_wifi.h"
#include "udp_log.h"

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <mdns.h>
#include <nvs_flash.h>

#include <optional>
#include <vector>

// clang-format off
std::optional<sys::wifisvc> g_wifi;
wifi_config_t g_wifi_config = {
    .sta = {
        .ssid = CONFIG_NGW_WIFI_SSID,
        .password = CONFIG_NGW_WIFI_PASSWORD,
        .threshold = {
            .authmode = WIFI_AUTH_WPA2_WPA3_PSK,
        },
        .pmf_cfg = { .capable = true, .required = false },
    }
};

std::optional<sys::nibeclient> g_gw;
sys::nibeclient::config_t g_gw_config = {
    .enable_necessary = true,
    .enable_pin = (gpio_num_t)CONFIG_NGW_UART_RS485_RTS,
    .uart_port = (uart_port_t)CONFIG_NGW_UART_RS485_PORT_NUM,
    .uart_tx_pin = (gpio_num_t)CONFIG_NGW_UART_RS485_TX,
    .uart_rx_pin = (gpio_num_t)CONFIG_NGW_UART_RS485_RX,
    .address = 0x0020
};

std::optional<sys::modbus> g_mb;
sys::modbus::config_t g_mb_config = { 
    .netif = nullptr
};
// clang-format on


constexpr const char* TAG = "nibegw.main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting...");
    ESP_ERROR_CHECK(nvs_flash_init()); // Wifi stack requires NVS.
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("nibegw"));
    ESP_ERROR_CHECK(mdns_instance_name_set("NIBE Modbus Gateway"));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_modbus", "_tcp", CONFIG_NGW_MODBUS_PORT, nullptr, 0));

    if (CONFIG_NGW_UDPLOG_ENABLE) {
        ESP_ERROR_CHECK(udp_log_init(CONFIG_NGW_UDPLOG_DST_ADDR, CONFIG_NGW_UDPLOG_DST_PORT));
        esp_log_set_vprintf(udp_log);
    }

    g_wifi = std::make_optional<sys::wifisvc>(g_wifi_config);
    ESP_ERROR_CHECK(g_wifi->start());

    g_mb_config.netif = g_wifi->get_netif();
    g_mb = std::make_optional<sys::modbus>(g_mb_config);

    g_gw_config.on_read_request = [&](uint16_t& to_read) {
        auto rq = g_mb->get_read_queue();
        auto rc = xQueueReceive(rq, &to_read, 0);
        if (rc == pdFALSE) {
            return false;
        }

        ESP_LOGI(TAG, "Requesting register: %u", to_read);
        return true;
    };

    g_gw_config.on_read_response = [&](sys::nibeclient::datapoint read) {
        // We always get two registers.
        ESP_LOGI(TAG, "Read response for register: %u, Value: %" PRIu32 "", read.reg,
            read.value & 0xFFFF);
        ESP_LOGI(TAG, "Read response for register: %u, Value: %" PRIu32 "", read.reg + 1,
            read.value >> 16);
        g_mb->update_register_u32(read.reg, read.value);
    };

    g_gw_config.on_write_request = [&](sys::nibeclient::datapoint& to_write) {
        auto wq = g_mb->get_write_queue();

        sys::modbus::write_request wrq;
        auto rc = xQueueReceive(wq, &wrq, 0);
        if (rc == pdFALSE) {
            return false;
        }

        to_write.reg = wrq.reg;
        to_write.value = wrq.value;

        if (CONFIG_NGW_ENABLE_WRITES) {
            ESP_LOGI(TAG, "Write request to register: %u, Value: %" PRIu32 "", to_write.reg,
                to_write.value);
            return true;
        } else {
            ESP_LOGI(TAG, "Write request to register: %u, Value: %" PRIu32 " - (DISABLED)",
                to_write.reg, to_write.value);
            return false;
        }
    };

    g_gw_config.on_write_response = [](uint16_t, bool success) {
        if (!success) {
            ESP_LOGW(TAG, "Last write request was rejected by the heatpump!");
        }
    };

    g_gw_config.on_periodic_update = [&](const std::vector<sys::nibeclient::datapoint>& points) {
        for (auto& dp : points) {
            ESP_LOGI(TAG, "Refresh of register: %u, Value: %" PRIu32 "", dp.reg, dp.value);
            g_mb->update_register_u16(dp.reg, (uint16_t)dp.value);
        }
        return true;
    };

    g_gw = std::make_optional<sys::nibeclient>(g_gw_config);

    ESP_ERROR_CHECK(g_gw->start());
    ESP_ERROR_CHECK(g_mb->start());

    ESP_LOGI(TAG, "Started!");

    while (true) {
        /* Keeping the local variables alive with this task.
           Maybe useful for other things too in the future. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
