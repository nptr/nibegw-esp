#include "udp_log.h"
#include "esp_err.h"

#include <lwip/err.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>


static int g_log_sock;
static struct sockaddr_in g_log_addr;
static char g_log_buffer[512];

esp_err_t udp_log_init(const char* addr, int port)
{
    g_log_addr.sin_addr.s_addr = inet_addr(addr);
    g_log_addr.sin_family = AF_INET;
    g_log_addr.sin_port = htons(port);

    g_log_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_log_sock < 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


int udp_log(const char* format, va_list args)
{
    int num_chars = vsprintf(g_log_buffer, format, args);
    if (num_chars <= 0) {
        return 0;
    }
    if (num_chars > 500)
        num_chars = 500;
    sendto(
        g_log_sock, g_log_buffer, num_chars, 0, (struct sockaddr*)&g_log_addr, sizeof(g_log_addr));
    return num_chars;
}
