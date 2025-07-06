#include "sys_modbus.h"
#include "sdkconfig.h"

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_modbus_common.h>
#include <esp_modbus_slave.h>

#include <cstddef>

namespace sys {

constexpr auto TAG = "sys.modbus";
constexpr size_t MAX_REGS = 10000;

static uint16_t g_holding_regs[MAX_REGS];
static SemaphoreHandle_t g_hregs_mutex; // overzealous locking..
static QueueHandle_t g_read_queue;
static QueueHandle_t g_write_queue;

/* HACK: Overriding `mbc_reg_holding_slave_cb` because the `esp-modbus` implementation
 * can't read u32 as big endian. Also, there seems to be no way to pass a user context
 * so I had to use globals `g_holding_regs`, `g_read_queue` and `g_write_queue` instead
 * of members. But none is going to instantiate this class another time anyways.
 */
extern "C" mb_err_enum_t mbc_reg_holding_slave_cb(mb_base_t* inst, uint8_t* reg_buffer,
    uint16_t address, uint16_t n_regs, mb_reg_mode_enum_t mode)
{
    if (address >= MAX_REGS) {
        return MB_ENOREG;
    }

    switch (mode) {
    case MB_REG_READ: {
        if (n_regs == 2) {
            xSemaphoreTake(g_hregs_mutex, portMAX_DELAY);
            *reg_buffer++ = g_holding_regs[address + 0] >> 8; // LW, HB
            *reg_buffer++ = g_holding_regs[address + 0] & 0xFF; // LW, LB
            *reg_buffer++ = g_holding_regs[address + 1] >> 8; // HW, HB
            *reg_buffer++ = g_holding_regs[address + 1] & 0xFF; // HW, LB
            xSemaphoreGive(g_hregs_mutex);
        } else if (n_regs == 1) {
            xSemaphoreTake(g_hregs_mutex, portMAX_DELAY);
            *reg_buffer++ = g_holding_regs[address + 0] >> 8; // HB
            *reg_buffer++ = g_holding_regs[address + 0] & 0xFF; // LB
            xSemaphoreGive(g_hregs_mutex);
        } else {
            return MB_EINVAL; // No arbitrary reads
        }

        modbus::read_request rrq;
        rrq.reg = address;
        if (xQueueSend(g_read_queue, &rrq, pdMS_TO_TICKS(1)) == pdPASS) {
            ESP_LOGI(TAG, "Reading holding register: %u, NRegs: %u", (unsigned)address,
                (unsigned)n_regs);
        } else {
            ESP_LOGI(TAG, "Reading holding register: %u, NRegs: %u - Queue is full.",
                (unsigned)address, (unsigned)n_regs);
        }
    } break;
    case MB_REG_WRITE:
        modbus::write_request wrq;
        wrq.reg = address;

        if (n_regs == 2) {
            xSemaphoreTake(g_hregs_mutex, portMAX_DELAY);
            g_holding_regs[address + 0] = reg_buffer[0] << 8 | reg_buffer[1];
            g_holding_regs[address + 1] = reg_buffer[2] << 8 | reg_buffer[3];
            wrq.value = g_holding_regs[address + 0] | g_holding_regs[address + 1] << 8;
            xSemaphoreGive(g_hregs_mutex);
        } else if (n_regs == 1) {
            xSemaphoreTake(g_hregs_mutex, portMAX_DELAY);
            g_holding_regs[address + 0] = reg_buffer[0] << 8 | reg_buffer[1];
            wrq.value = g_holding_regs[address + 0];
            xSemaphoreGive(g_hregs_mutex);
        } else {
            return MB_EINVAL; // No arbitrary reads
        }

        if (xQueueSend(g_write_queue, &wrq, pdMS_TO_TICKS(1)) == pdPASS) {
            ESP_LOGI(TAG, "Writing holding register: %u, NRegs: %u", (unsigned)address,
                (unsigned)n_regs);
        } else {
            ESP_LOGI(TAG, "Writing holding register: %u, NRegs: %u - Queue is full.",
                (unsigned)address, (unsigned)n_regs);
        }

        break;
    }
    return MB_ENOERR;
}


modbus::modbus(config_t cfg)
    : m_config(cfg)
    , m_slave(nullptr)
{
    g_hregs_mutex = xSemaphoreCreateMutex();
    g_read_queue = xQueueCreate(20, sizeof(read_request));
    g_write_queue = xQueueCreate(20, sizeof(write_request));
}


esp_err_t modbus::start()
{
    mb_communication_info_t mbconf
        = { .tcp_opts = { .mode = MB_TCP, // mode of communication for slave
                .port = CONFIG_NGW_MODBUS_PORT, // communication port number for Modbus slave
                .uid = CONFIG_NGW_MODBUS_ADDR, // Modbus slave Unit Identifier
                .response_tout_ms = 0,
                .addr_type = MB_IPV4, // type of addressing being used
                .ip_addr_table = NULL, // Bind to any address
                .ip_netif_ptr = (void*)m_config.netif, // the pointer to netif inteface
                .dns_name = NULL,
                .start_disconnected = false } };
    esp_err_t ec = mbc_slave_create_tcp(&mbconf, &m_slave);
    if (m_slave == NULL || ec != ESP_OK) {
        return ec;
    }

    ec = mbc_slave_start(m_slave);
    if (ec != ESP_OK) {
        return ec;
    }

    return ESP_OK;
}


QueueHandle_t modbus::get_read_queue() const { return g_read_queue; }
QueueHandle_t modbus::get_write_queue() const { return g_write_queue; };


void modbus::update_register_u16(uint16_t reg, uint16_t value)
{
    if (reg >= MAX_REGS) {
        return;
    }

    xSemaphoreTake(g_hregs_mutex, portMAX_DELAY);
    g_holding_regs[reg] = value;
    xSemaphoreGive(g_hregs_mutex);
}


void modbus::update_register_u32(uint16_t reg, uint32_t value)
{
    if (reg - 1 >= MAX_REGS) {
        return;
    }

    xSemaphoreTake(g_hregs_mutex, portMAX_DELAY);
    g_holding_regs[reg] = value & 0xFFFF;
    g_holding_regs[reg + 1] = (value >> 16) & 0xFFFF;
    xSemaphoreGive(g_hregs_mutex);
}
}