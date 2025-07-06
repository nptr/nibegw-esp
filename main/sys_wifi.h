#pragma once

#include <esp_err.h>
#include <esp_wifi.h>

namespace sys {

extern "C" void wifi_eventhandler(void*, esp_event_base_t, int32_t, void*);
extern "C" void ip_eventhandler(void*, esp_event_base_t, int32_t, void*);

class wifisvc {
public:
    wifisvc(wifi_config_t config);
    esp_err_t start();

    esp_netif_t* get_netif() const;

private:
    friend void wifi_eventhandler(void*, esp_event_base_t, int32_t, void*);
    friend void ip_eventhandler(void*, esp_event_base_t, int32_t, void*);

    wifi_config_t m_config;
    esp_netif_t* m_netif;
    TimerHandle_t m_rc_timer;
};

}