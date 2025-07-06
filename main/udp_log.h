#pragma once

#include <esp_err.h>

esp_err_t udp_log_init(const char* addr, int port);
int udp_log(const char* format, va_list args);