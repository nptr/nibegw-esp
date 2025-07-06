#include "sys_wifi.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_wifi.h>


namespace sys {

constexpr auto TAG = "sys.wifi";


extern "C" void wifi_eventhandler(
    void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Connection failed. Attempting reconnect in 10s.");
        wifisvc* instance = static_cast<wifisvc*>(arg);
        xTimerStart(instance->m_rc_timer, 0);
    }
}


extern "C" void ip_eventhandler(
    void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base != IP_EVENT) {
        return;
    }

    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}


void wifi_reconnect(TimerHandle_t timer)
{
    esp_err_t ec = esp_wifi_connect();
    if (ec != ESP_OK) {
        ESP_LOGI(
            TAG, "esp_wifi_connect() failed: %s. Attempting again in 10s.", esp_err_to_name(ec));
        xTimerStart(timer, 0);
    } else {
        // ESP_OK doesn't mean the connection succeeded!
    }
}


wifisvc::wifisvc(wifi_config_t config)
    : m_config(config)
{
    m_rc_timer = xTimerCreate("wifi_rc", pdMS_TO_TICKS(10000), pdFALSE, nullptr, &wifi_reconnect);
}


esp_err_t wifisvc::start()
{
    m_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ec = esp_wifi_init(&cfg);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init(): %s", esp_err_to_name(ec));
        return ec;
    }

    esp_event_handler_instance_t wifi_evt_handler;
    ec = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_eventhandler, this, &wifi_evt_handler);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_handler_instance_register(): %s", esp_err_to_name(ec));
        return ec;
    }

    esp_event_handler_instance_t ip_evt_handler;
    ec = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_eventhandler, this, &ip_evt_handler);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_handler_instance_register(): %s", esp_err_to_name(ec));
        return ec;
    }

    ec = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(): %s", esp_err_to_name(ec));
        return ec;
    }

    ec = esp_wifi_set_config(WIFI_IF_STA, &m_config);
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(): %s", esp_err_to_name(ec));
        return ec;
    }

    ec = esp_wifi_start();
    if (ec != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start(): %s", esp_err_to_name(ec));
        return ec;
    }

    return ESP_OK;
}


esp_netif_t* wifisvc::get_netif() const { return m_netif; }

}