#pragma once

#include "esp_err.h"
#include "esp_zigbee_core.h"

/* ============================================================
 *  Zigbee Role Selection
 * ============================================================ */
typedef enum 
{
    ZB_ROLE_COORDINATOR = 0,
    ZB_ROLE_END_DEVICE  = 1,
} zb_role_t;


/* ============================================================
 *  STM32 ZB Command Definitions
 * ============================================================ */
typedef enum 
{
    ZB_CMD_OK,                  // 지그비 기본 응답
    ZB_CMD_PING,                // 코디네이터 핑퐁 시작
    ZB_CMD_RESET,
    ZB_CMD_PERMIT_OPEN,         // 코디네이터 네트워크 조인 허용 시작
    ZB_CMD_PERMIT_CLOSE,        // 코디네이터 네트워크 조인 허용 종료
    ZB_CMD_ROLE_COORD,                // 지그비 역할 설정
    ZB_CMD_ROLE_ED,                   // 지그비 역할 설정
    ZB_CMD_JOIN,                // 엔드 디바이스 네트워크 조인 시작
} zb_cmd_id_t;


/* ============================================================
 *  Common Zigbee Configuration
 * ============================================================ */
#define INSTALLCODE_POLICY_ENABLE       false
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)
#define APP_PROD_CFG_CURRENT_VERSION    0x0001

/* ============================================================
 *  Manufacturer Information
 * ============================================================ */
#define ESP_MANUFACTURER_CODE   0x131B
#define ESP_MANUFACTURER_NAME   "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER    "\x07"CONFIG_IDF_TARGET

/* ============================================================
 *  Coordinator Specific Configuration
 * ============================================================ */
#define ZB_COORD_MAX_CHILDREN   10
#define ZB_COORD_ENDPOINT       1

#define ESP_ZB_COORDINATOR_CONFIG()                         \
{                                                           \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,          \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
    .nwk_cfg.zczr_cfg = {                                   \
        .max_children = ZB_COORD_MAX_CHILDREN,              \
    },                                                      \
}

/* ============================================================
 *  End Device Specific Configuration
 * ============================================================ */
#define ZB_ED_ENDPOINT          10

#define ESP_ZB_END_DEVICE_CONFIG()                          \
{                                                           \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                   \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
    .nwk_cfg.zed_cfg = {                                    \
        .keep_alive      = 3000,                            \
        .ed_timeout      = ESP_ZB_ED_AGING_TIMEOUT_64MIN,   \
    },                                                      \
}

/* ============================================================
 *  RCP / Radio Configuration (Shared)
 * ============================================================ */
#define HOST_RX_PIN_TO_RCP_TX   4
#define HOST_TX_PIN_TO_RCP_RX   5

#if CONFIG_ZB_RADIO_NATIVE

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                       \
{                                                           \
    .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
}

#else   /* UART RCP */

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                       \
{                                                           \
    .radio_mode = ZB_RADIO_MODE_UART_RCP,                   \
    .radio_uart_config = {                                  \
        .port = 1,                                          \
        .uart_config = {                                    \
            .baud_rate = 460800,                            \
            .data_bits = UART_DATA_8_BITS,                  \
            .parity    = UART_PARITY_DISABLE,               \
            .stop_bits = UART_STOP_BITS_1,                  \
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,          \
            .rx_flow_ctrl_thresh = 0,                       \
            .source_clk = UART_SCLK_DEFAULT,                \
        },                                                  \
        .rx_pin = HOST_RX_PIN_TO_RCP_TX,                    \
        .tx_pin = HOST_TX_PIN_TO_RCP_RX,                    \
    },                                                      \
}

#endif

/* ============================================================
 *  Host Configuration (Shared)
 * ============================================================ */
#define ESP_ZB_DEFAULT_HOST_CONFIG()                        \
{                                                           \
    .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
}

/* ============================================================
 *  Helper: Build esp_zb_cfg_t by role
 * ============================================================ */
static inline esp_zb_cfg_t zb_build_cfg_by_role(zb_role_t role)
{
    esp_zb_cfg_t cfg = {0};

    if (role == ZB_ROLE_COORDINATOR) 
    {
        cfg = (esp_zb_cfg_t)ESP_ZB_COORDINATOR_CONFIG();
    } 
    else 
    {
        cfg = (esp_zb_cfg_t)ESP_ZB_END_DEVICE_CONFIG();
    }

    return cfg;
}
