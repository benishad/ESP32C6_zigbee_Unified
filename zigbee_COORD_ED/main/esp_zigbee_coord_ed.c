/******************************************************************************
 *  ESP32-C6 Zigbee Unified Coordinator / End Device
 *  - One firmware
 *  - ZB+ROLE,COORD / ZB+ROLE,ED
 *  - Reboot-based role switch
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_timer.h"

#include "esp_zigbee_core.h"
#include "zdo/esp_zigbee_zdo_common.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "nwk/esp_zigbee_nwk.h"
#include "aps/esp_zigbee_aps.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "esp_zigbee_coord_ed.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* LOG TAG */
/* ========================================================================== */
static const char *TAG = "ZB_UNIFIED";

/* ========================================================================== */
/* UART (STM32) */
/* ========================================================================== */
#define UART_STM32      UART_NUM_1
#define UART_TX_PIN    GPIO_NUM_6
#define UART_RX_PIN    GPIO_NUM_7
#define UART_RX_BUF    256

static void stm_uart_send(const char *s)
{
    if (s) uart_write_bytes(UART_STM32, s, strlen(s));
}



/* ========================================================================== */
/* dip switch */
/* ========================================================================== */
#define ZB_SW_GPIO            GPIO_NUM_9
#define ZB_SW_ACTIVE_LEVEL    0

#define ZB_SW_DEBOUNCE_MS     30
#define ZB_SW_POLL_MS         50

#define ZB_SW_LONG_PRESS_MS  5000   // 5초




/* ========================================================================== */
/* Zigbee ROLE 관리 (NVS) */
/* ========================================================================== */

#define NVS_NS_ZB     "zb_cfg"
#define NVS_KEY_ROLE  "role"

static zb_role_t zb_load_role(void)
{
    nvs_handle_t h;
    uint8_t v = ZB_ROLE_COORDINATOR;

    if (nvs_open(NVS_NS_ZB, NVS_READONLY, &h) == ESP_OK) 
    {
        nvs_get_u8(h, NVS_KEY_ROLE, &v);
        nvs_close(h);
    }
    return (zb_role_t)v;
}

static void zb_save_role(zb_role_t role)
{
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NS_ZB, NVS_READWRITE, &h));
    ESP_ERROR_CHECK(nvs_set_u8(h, NVS_KEY_ROLE, role));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
}


/* ========================================================================== */
/* Zigbee SW ISR */
/* ========================================================================== */
static QueueHandle_t s_sw_q;

static void IRAM_ATTR zb_sw_isr(void *arg)
{
    uint32_t gpio = (uint32_t)arg;
    xQueueSendFromISR(s_sw_q, &gpio, NULL);
}



/* ========================================================================== */
/* Zigbee Custom Cluster */
/* ========================================================================== */
#define ZCL_CLUSTER_PING     0xFC00
#define COORD_EP             1
#define ED_EP                10


/* ========================================================================== */
/* End Device Management (Coordinator only) */
/* ========================================================================== */
#define MAX_ED 20

#define PING_INTERVAL_MS        (10 * 1000)

typedef struct {
    bool     used;
    uint16_t short_addr;
    uint64_t ieee_addr;

    int64_t  last_pong_us;
    uint8_t  miss_count;
    bool     alive;
} ed_node_t;

static ed_node_t s_ed_table[MAX_ED];

/* ========================================================================== */
/* End Device Table Utils (Coordinator only) */
/* ========================================================================== */
static ed_node_t *find_ed(uint16_t short_addr)
{
    for (int i = 0; i < MAX_ED; i++) 
    {
        if (s_ed_table[i].used && s_ed_table[i].short_addr == short_addr) 
        {
            return &s_ed_table[i];
        }
    }
    return NULL;
}

static ed_node_t *alloc_ed(uint16_t short_addr, uint64_t ieee)
{
    for (int i = 0; i < MAX_ED; i++) 
    {
        if (!s_ed_table[i].used) 
        {
            s_ed_table[i].used = true;
            s_ed_table[i].short_addr = short_addr;
            s_ed_table[i].ieee_addr  = ieee;
            s_ed_table[i].alive = true;
            s_ed_table[i].miss_count = 0;
            s_ed_table[i].last_pong_us = esp_timer_get_time();
            return &s_ed_table[i];
        }
    }
    return NULL;
}



/* ========================================================================== */
/* End Device Health Check Callback */
/* ========================================================================== */
#define ED_TIMEOUT_US   (30 * 1000000LL)
#define ED_MAX_MISS     3

static void ed_health_check_cb(uint8_t dummy)
{
    int64_t now = esp_timer_get_time();

    for (int i = 0; i < MAX_ED; i++) 
    {
        ed_node_t *ed = &s_ed_table[i];
        if (!ed->used || !ed->alive) 
        {
            continue;
        }

        if (now - ed->last_pong_us > ED_TIMEOUT_US) 
        {
            ed->miss_count++;

            if (ed->miss_count >= ED_MAX_MISS) 
            {
                ed->alive = false;

                ESP_LOGW(TAG, "ED DEAD: short=0x%04X ieee=%016llX", ed->short_addr, ed->ieee_addr);

                // STM32 통보 (선택)
                char buf[64];
                snprintf(buf, sizeof(buf), "ZB+ED,LOST,0x%04X,%016llX\r\n", ed->short_addr, ed->ieee_addr);
                stm_uart_send(buf);
            }
        }
    }

    // 5초마다 반복
    esp_zb_scheduler_alarm(ed_health_check_cb, 0, 5000);
}




/* ================= Coordinator-only function prototypes ================= */
static void aps_ping_task(uint8_t dummy);
static void permit_join_apply_cb(uint8_t enable);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static void zb_switch_task(void *arg);




/* ========================================================================== */
/* STM32 ZB Command Definitions */
/* ========================================================================== */

typedef void (*zb_cmd_handler_t)(const char *args);

typedef struct 
{
    const char *cmd;
    zb_cmd_id_t id;
    zb_cmd_handler_t handler;
} zb_cmd_entry_t;

/* ========================================================================== */
/* STM32 ZB Command Handlers */
/* ========================================================================== */
static bool s_ping_running = false;
static bool s_health_running = false;

static void zb_cmd_ok(const char *args)
{
    stm_uart_send("ZB\r\n");
    stm_uart_send("ZB+OK\r\n");

    // 보낸 메세지 로그에 출력
    ESP_LOGI(TAG, "UART->STM32: ZB ZB+OK");
}

static void zb_cmd_ping(const char *args)
{
    if (zb_load_role() != ZB_ROLE_COORDINATOR) 
    {
        stm_uart_send("ZB+ERR,ROLE,NOT_COORDINATOR\r\n");
        return;
    }

    if (!s_ping_running) 
    {
        // 지그비에서는 중복으로 실행되며 CPU 부하가 커질 수 있으므로 플래그로 한번만 실행하게 제어
        esp_zb_scheduler_alarm(aps_ping_task, 0, 0);
        s_ping_running = true;
    }

    if (!s_health_running) 
    {
        // 지그비에서는 중복으로 실행되며 CPU 부하가 커질 수 있으므로 플래그로 한번만 실행하게 제어
        esp_zb_scheduler_alarm(ed_health_check_cb, 0, 5000);
        s_health_running = true;
    }

    stm_uart_send("ZB\r\n");
    stm_uart_send("ZB+PING_OK\r\n");
    // 보낸 메세지 로그에 출력
    ESP_LOGI(TAG, "UART->STM32: ZB ZB+PING_OK");
}

static void zb_cmd_reset(const char *args)
{
    stm_uart_send("ZB+RESETTING\r\n");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

static void zb_cmd_permit_open(const char *args)
{
    if (zb_load_role() != ZB_ROLE_COORDINATOR)
        return;

    esp_zb_scheduler_alarm( (esp_zb_callback_t)permit_join_apply_cb, 1, 0 );

    stm_uart_send("ZB\r\n");
    stm_uart_send("ZB+OPEN_OK\r\n");
    // 보낸 메세지 로그에 출력
    ESP_LOGI(TAG, "UART->STM32: ZB ZB+OPEN_OK");
}

static void zb_cmd_permit_close(const char *args)
{
    if (zb_load_role() != ZB_ROLE_COORDINATOR)
        return;

    esp_zb_scheduler_alarm(
        (esp_zb_callback_t)permit_join_apply_cb,
        0,
        0
    );
}

static void zb_cmd_role_coord(const char *args)
{
    (void)args;

    zb_role_t cur = zb_load_role();
    if (cur == ZB_ROLE_COORDINATOR) 
    {
        stm_uart_send("ZB\r\n");
        stm_uart_send("ZB+ROLE,COORD,ALREADY\r\n");
        ESP_LOGI(TAG, "ROLE already COORDINATOR");
        return;
    }

    zb_save_role(ZB_ROLE_COORDINATOR);

    stm_uart_send("ZB\r\n");
    stm_uart_send("ZB+ROLE,COORD,OK\r\n");

    ESP_LOGW(TAG, "ROLE set to COORDINATOR, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

static void zb_cmd_role_ed(const char *args)
{
    (void)args;

    zb_role_t cur = zb_load_role();
    if (cur == ZB_ROLE_END_DEVICE) 
    {
        stm_uart_send("ZB\r\n");
        stm_uart_send("ZB+ROLE,ED,ALREADY\r\n");
        ESP_LOGI(TAG, "ROLE already END DEVICE");
        return;
    }

    zb_save_role(ZB_ROLE_END_DEVICE);

    stm_uart_send("ZB\r\n");
    stm_uart_send("ZB+ROLE,ED,OK\r\n");

    ESP_LOGW(TAG, "ROLE set to END DEVICE, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

static void zb_cmd_join(const char *args)
{
    (void)args;

    if (zb_load_role() != ZB_ROLE_END_DEVICE) 
    {
        stm_uart_send("ZB+ERR,ROLE,NOT_ED\r\n");
        return;
    }

    // Zigbee stack이 아직 준비 안 됐을 수도 있음
    if (!esp_zb_is_started()) 
    {
        stm_uart_send("ZB+ERR,ZB_NOT_READY\r\n");
        return;
    }

    // factory-new가 아니면 steering만 허용할지 정책 선택
    // bool factory_new = esp_zb_bdb_is_factory_new();
    // if (!factory_new) 
    // {
    //     stm_uart_send("ZB+ERR,NOT_FACTORY_NEW\r\n");
    //     return;
    // }

    // Zigbee context에서 commissioning 시작
    esp_zb_scheduler_alarm(
        (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
        ESP_ZB_BDB_MODE_NETWORK_STEERING,
        0
    );

    stm_uart_send("ZB\r\n");
    stm_uart_send("ZB+JOIN,OK\r\n");

    ESP_LOGI(TAG, "JOIN START (NETWORK STEERING)");
}



/* ========================================================================== */
/* Factory Reset */
/* ========================================================================== */
static void zigbee_factory_reset(void)
{
    ESP_LOGW(TAG, "ZIGBEE FACTORY RESET requested");

    /* Zigbee stack 즉시 factory reset */
    esp_zb_factory_reset();

    // Zigbee NVRAM을 다음 부팅 시 erase
    esp_zb_nvram_erase_at_start(true);

    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}


/* ========================================================================== */
/* Zigbee Switch Init */
/* ========================================================================== */
static void zb_switch_init(void)
{
    s_sw_q = xQueueCreate(4, sizeof(uint32_t));

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ZB_SW_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ZB_SW_GPIO, zb_sw_isr, (void *)ZB_SW_GPIO);

    xTaskCreate(zb_switch_task, "zb_sw", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "Zigbee switch init: GPIO=%d", ZB_SW_GPIO);
}




/* ========================================================================== */
/* APS RX Callback (시그니처 중요) */
/* ========================================================================== */
static bool aps_rx_cb(struct esp_zb_apsde_data_ind_s ind)
{
    if (!ind.asdu || ind.asdu_length < 7)
        return false;

    if (ind.cluster_id != ZCL_CLUSTER_PING)
        return false;

    /* Coordinator 수신 */
    if (memcmp(ind.asdu, "ZB+PONG", 7) == 0) 
    {
        ed_node_t *ed = find_ed(ind.src_short_addr);
        if (ed) 
        {
            ed->last_pong_us = esp_timer_get_time();
            ed->miss_count  = 0;
            ed->alive       = true;
        }
        
        char buf[64];
        snprintf(buf, sizeof(buf), "ZB+PONG,0x%04X\r\n", ind.src_short_addr);
        stm_uart_send(buf);

        ESP_LOGI(TAG, "PONG from 0x%04X", ind.src_short_addr);
        return true;
    }

    /* End Device 수신 */
    if (memcmp(ind.asdu, "ZB+PING", 7) == 0) 
    {
        static uint8_t pong[] = { 'Z','B','+','P','O','N','G' };

        esp_zb_apsde_data_req_t req = {0};
        req.dst_addr_mode       = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        req.dst_addr.addr_short = ind.src_short_addr;
        req.dst_endpoint        = ind.src_endpoint;
        req.src_endpoint        = ED_EP;
        req.profile_id          = ESP_ZB_AF_HA_PROFILE_ID;
        req.cluster_id          = ZCL_CLUSTER_PING;
        req.asdu                = pong;
        req.asdu_length         = sizeof(pong);

        esp_zb_aps_data_request(&req);
        ESP_LOGI(TAG, "PING received -> PONG sent");
        return true;
    }

    return false;
}

/* ========================================================================== */
/* Zigbee Signal Handler */
/* ========================================================================== */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    if (!signal_struct || !signal_struct->p_app_signal)
        return;

    esp_zb_app_signal_type_t sig = (esp_zb_app_signal_type_t)(*signal_struct->p_app_signal);

    switch (sig) 
    {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack ready");

            if (zb_load_role() == ZB_ROLE_COORDINATOR) 
            {
                esp_zb_scheduler_alarm(
                    (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                    ESP_ZB_BDB_MODE_NETWORK_FORMATION,
                    0
                );
            }
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            ESP_LOGI(TAG, "Zigbee start");
            break;

        case ESP_ZB_BDB_SIGNAL_FORMATION:
            ESP_LOGI(TAG, "Network formation done");
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            ESP_LOGI(TAG, "Network steering done");
            break;
        
        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: 
        {
            const esp_zb_zdo_signal_device_annce_params_t *p =
                (const esp_zb_zdo_signal_device_annce_params_t *)
                esp_zb_app_signal_get_params(signal_struct->p_app_signal);

            if (!p) break;

            uint64_t ieee = 0;
            for (int i = 0; i < 8; i++) 
            {
                ieee = (ieee << 8) | p->ieee_addr[i];
            }

            ed_node_t *ed = find_ed(p->device_short_addr);

            if (!ed) 
            {
                // NEW JOIN
                ed = alloc_ed(p->device_short_addr, ieee);

                ESP_LOGI(TAG,
                    "ED JOIN NEW: short=0x%04X ieee=%016llX",
                    p->device_short_addr, ieee
                );

                // STM32 알림
                char buf[64];
                snprintf(buf, sizeof(buf), "ZB+ED,NEW,0x%04X,%016llX\r\n", p->device_short_addr, ieee);
                stm_uart_send(buf);

            } 
            else 
            {
                // REJOIN
                ed->alive = true;
                ed->miss_count = 0;
                ed->last_pong_us = esp_timer_get_time();

                ESP_LOGI(TAG, "ED REJOIN: short=0x%04X ieee=%016llX", p->device_short_addr, ieee );

                char buf[64];
                snprintf(buf, sizeof(buf), "ZB+ED,REJOIN,0x%04X,%016llX\r\n", p->device_short_addr, ieee);
                stm_uart_send(buf);
            }

            break;
        }



        default:
            ESP_LOGD(TAG, "Unhandled Zigbee signal: %d", sig);
            break;
    }
}


/* ========================================================================== */
/* Dip Switch Task */
/* ========================================================================== */
static void zb_switch_task(void *arg)
{
    uint32_t io;
    int level;
    int press_time = 0;
    bool pressing = false;

    zb_role_t role = zb_load_role();

    ESP_LOGI(TAG, "Zigbee switch task started (role=%s)", role == ZB_ROLE_COORDINATOR ? "COORD" : "ED");

    while (1)
    {
        if (xQueueReceive(s_sw_q, &io, portMAX_DELAY))
        {
            vTaskDelay(pdMS_TO_TICKS(ZB_SW_DEBOUNCE_MS));
            level = gpio_get_level(ZB_SW_GPIO);

            if (level == ZB_SW_ACTIVE_LEVEL && !pressing)
            {
                pressing = true;
                press_time = 0;
                ESP_LOGI(TAG, "ZB SW DOWN");
            }
        }

        while (pressing)
        {
            vTaskDelay(pdMS_TO_TICKS(ZB_SW_POLL_MS));
            level = gpio_get_level(ZB_SW_GPIO);

            if (level != ZB_SW_ACTIVE_LEVEL)
            {
                ESP_LOGI(TAG, "ZB SW RELEASE (%d ms)", press_time);
                pressing = false;
                press_time = 0;
                break;
            }

            press_time += ZB_SW_POLL_MS;

            if (press_time >= ZB_SW_LONG_PRESS_MS)
            {
                ESP_LOGW(TAG, "ZB SW LONG PRESS (%d ms) -> FACTORY RESET", press_time);

                zigbee_factory_reset();
            }
        }
    }
}



/* ========================================================================== */
/* Coordinator-only APS PING Task */
/* ========================================================================== */
static void aps_ping_task(uint8_t dummy)
{
    (void)dummy;

    static int idx = 0;
    int start = idx;

    if (zb_load_role() != ZB_ROLE_COORDINATOR)
        return;

    while (1)
    {
        ed_node_t *ed = &s_ed_table[idx];
        idx = (idx + 1) % MAX_ED;

        if (ed->used && ed->alive)
        {
            esp_zb_apsde_data_req_t req = {0};

            static uint8_t payload[] = { 'Z', 'B', '+', 'P', 'I', 'N', 'G' };

            req.dst_addr_mode       = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            req.dst_addr.addr_short = ed->short_addr;
            req.dst_endpoint        = ED_EP;
            req.src_endpoint        = COORD_EP;
            req.profile_id          = ESP_ZB_AF_HA_PROFILE_ID;
            req.cluster_id          = ZCL_CLUSTER_PING;
            req.asdu                = payload;
            req.asdu_length         = sizeof(payload);
            req.tx_options          = 0;
            req.radius              = 0;

            ESP_LOGI(TAG, "ZB+PING -> ED 0x%04x", ed->short_addr);
            esp_zb_aps_data_request(&req);

            break;
        }

        if (idx == start) break;
    }

    esp_zb_scheduler_alarm(aps_ping_task, 0, PING_INTERVAL_MS);
}



/* ========================================================================== */
/* Coordinator Permit-Join Control */
/* ========================================================================== */

#define PERMIT_JOIN_SECONDS 60

static void permit_join_apply(uint8_t enable)
{
    esp_zb_zdo_permit_joining_req_param_t req = {0};

    req.dst_nwk_addr    = 0x0000;   // Trust Center (Coordinator)
    req.tc_significance = 1;
    req.permit_duration = enable ? PERMIT_JOIN_SECONDS : 0;

    esp_zb_zdo_permit_joining_req(&req, NULL, NULL);
}

static void permit_join_apply_cb(uint8_t enable)
{
    permit_join_apply(enable);

    if (enable) 
    {
        ESP_LOGI(TAG, "Permit-join OPEN (%ds)", PERMIT_JOIN_SECONDS);
    } 
    else 
    {
        ESP_LOGI(TAG, "Permit-join CLOSE");
    }
}

/* ========================================================================== */
/* BDB Commissioning Helper */
/* ========================================================================== */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
    if (err != ESP_OK) 
    {
        ESP_LOGW(TAG,
            "bdb_start_top_level_commissioning failed: %s (mode=0x%02x)",
            esp_err_to_name(err), mode_mask);
    }
}



/* ========================================================================== */
/* Coordinator Start */
/* ========================================================================== */
static void zigbee_start_coordinator(void)
{
    ESP_LOGW(TAG, "Starting as COORDINATOR");

    esp_zb_cfg_t cfg = zb_build_cfg_by_role(ZB_ROLE_COORDINATOR);
    esp_zb_init(&cfg);

    esp_zb_on_off_switch_cfg_t sw_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();

    esp_zb_ep_list_t *ep = esp_zb_on_off_switch_ep_create(COORD_EP, &sw_cfg);

    esp_zb_device_register(ep);
    esp_zb_aps_data_indication_handler_register(aps_rx_cb);

    esp_zb_set_primary_network_channel_set( ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK );

    esp_zb_start(false);
    esp_zb_stack_main_loop();
}

/* ========================================================================== */
/* End Device Start */
/* ========================================================================== */
static void zigbee_start_end_device(void)
{
    ESP_LOGW(TAG, "Starting as END DEVICE");

    esp_zb_cfg_t cfg = zb_build_cfg_by_role(ZB_ROLE_END_DEVICE);
    esp_zb_init(&cfg);

    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();

    esp_zb_ep_list_t *ep = esp_zb_on_off_light_ep_create(ED_EP, &light_cfg);

    esp_zb_device_register(ep);
    esp_zb_aps_data_indication_handler_register(aps_rx_cb);

    esp_zb_set_primary_network_channel_set( ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK );

    esp_zb_start(false);
    esp_zb_stack_main_loop();
}

/* ========================================================================== */
/* Zigbee Task */
/* ========================================================================== */
static void zigbee_task(void *arg)
{
    zb_role_t role = zb_load_role();

    esp_zb_platform_config_t platform = 
    {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform));

    if (role == ZB_ROLE_COORDINATOR)
    {
        zigbee_start_coordinator();
    }
    else
    {
        zigbee_start_end_device();
    }
}



/* ========================================================================== */
/* STM32 ZB Command Table & Parser */
/* ========================================================================== */

static const zb_cmd_entry_t zb_cmd_table[] = 
{
    { "ZB+OK",            ZB_CMD_OK,            zb_cmd_ok           },
    { "ZB+PING,START",    ZB_CMD_PING,          zb_cmd_ping         },
    { "ZB+RESET",         ZB_CMD_RESET,         zb_cmd_reset        },
    { "ZB+PERMIT,OPEN",   ZB_CMD_PERMIT_OPEN,   zb_cmd_permit_open  },
    { "ZB+PERMIT,CLOSE",  ZB_CMD_PERMIT_CLOSE,  zb_cmd_permit_close },
    { "ZB+ROLE,COORD",    ZB_CMD_ROLE_COORD,    zb_cmd_role_coord   },
    { "ZB+ROLE,ED",       ZB_CMD_ROLE_ED,       zb_cmd_role_ed      },
    { "ZB+JOIN",          ZB_CMD_JOIN,          zb_cmd_join         },
};

#define ZB_CMD_TABLE_SIZE (sizeof(zb_cmd_table)/sizeof(zb_cmd_table[0]))

static void zb_command_parse(char *line)
{
    char *cr = strpbrk(line, "\r\n");
    if (cr) *cr = '\0';

    for (int i = 0; i < ZB_CMD_TABLE_SIZE; i++) 
    {
        if (strncmp(line, zb_cmd_table[i].cmd, strlen(zb_cmd_table[i].cmd)) == 0)
        {
            const char *args = NULL;
            char *comma = strchr(line, ',');
            if (comma)
                args = comma + 1;

            zb_cmd_table[i].handler(args);
            return;
        }
    }

    stm_uart_send("ZB+ERR,UNKNOWN\r\n");
}



/* ========================================================================== */
/* STM32 UART RX (ZB+ROLE) */
/* ========================================================================== */
static void stm32_uart_task(void *arg)
{
    uint8_t buf[UART_RX_BUF];

    while (1)
    {
        int len = uart_read_bytes(UART_STM32, buf, sizeof(buf) - 1, pdMS_TO_TICKS(1000));
        if (len <= 0)
            continue;

        buf[len] = 0;

        // 수신받은 내용을 로그에 출력
        ESP_LOGI(TAG, "UART<-STM32: [%s]", (char *)buf);

        if (strncmp((char *)buf, "ZB+ROLE,", 8) == 0) 
        {
            if (strncmp((char *)buf, "ZB+ROLE,ED", 10) == 0) 
            {
                zb_save_role(ZB_ROLE_END_DEVICE);
                stm_uart_send("ZB+ROLE,OK\r\n");
                esp_restart();
            }
            else if (strncmp((char *)buf, "ZB+ROLE,COORD", 13) == 0) 
            {
                zb_save_role(ZB_ROLE_COORDINATOR);
                stm_uart_send("ZB+ROLE,OK\r\n");
                esp_restart();
            }
        }
        else 
        {
            zb_command_parse((char *)buf);
        }

    }
}

/* ========================================================================== */
/* app_main */
/* ========================================================================== */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    uart_config_t cfg = 
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_STM32, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_STM32, &cfg);
    uart_set_pin(UART_STM32, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    zb_switch_init();

    xTaskCreate(stm32_uart_task, "stm32_uart", 4096, NULL, 5, NULL);
    xTaskCreate(zigbee_task, "zigbee", 8192, NULL, 5, NULL);
}

#ifdef __cplusplus
}
#endif
