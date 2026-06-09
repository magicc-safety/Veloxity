#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_crc.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define BRIDGE_MAGIC 0x56555832u
#define BRIDGE_VERSION 2u
#define BRIDGE_FLAG_NONE 0u
#define EXTERNAL_UART UART_NUM_1
#define LOCAL_IO_BUFFER_SIZE CONFIG_BRIDGE_PACKET_PAYLOAD_MAX

#if CONFIG_BRIDGE_PACKET_PAYLOAD_MAX > ESP_NOW_MAX_DATA_LEN
#error "CONFIG_BRIDGE_PACKET_PAYLOAD_MAX exceeds ESP-NOW maximum packet length"
#endif

typedef enum {
    ROLE_GROUND = 1,
    ROLE_AIR = 2,
} bridge_role_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t seq;
    uint16_t len;
    uint16_t crc;
    uint8_t version;
    uint8_t role;
    uint8_t flags;
    uint8_t reserved;
    uint8_t payload[CONFIG_BRIDGE_PACKET_PAYLOAD_MAX];
} bridge_packet_t;

typedef struct {
    uint16_t len;
    uint8_t bytes[CONFIG_BRIDGE_PACKET_PAYLOAD_MAX];
} bridge_item_t;

static QueueHandle_t s_outbound_queue;
static QueueHandle_t s_inbound_queue;
static SemaphoreHandle_t s_send_done_sem;
static volatile esp_now_send_status_t s_last_send_status = ESP_NOW_SEND_FAIL;

static volatile uint32_t s_send_ok;
static volatile uint32_t s_send_fail;
static volatile uint32_t s_send_timeout;
static volatile uint32_t s_rx_packets;
static volatile uint32_t s_rx_drops;
static volatile uint32_t s_rx_bad_crc;
static volatile uint32_t s_rx_bad_peer;
static volatile uint32_t s_rx_seq_gaps;
static volatile uint32_t s_local_rx_bytes;
static volatile uint32_t s_local_tx_bytes;
static volatile uint32_t s_espnow_tx_bytes;
static volatile uint32_t s_espnow_rx_bytes;
static uint16_t s_tx_seq;
static uint16_t s_last_rx_seq;
static bool s_have_last_rx_seq;
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];
static uint8_t s_local_sta_mac[ESP_NOW_ETH_ALEN];

#if CONFIG_BRIDGE_ROLE_AIR
static const bridge_role_t LOCAL_ROLE = ROLE_AIR;
static const bridge_role_t PEER_ROLE = ROLE_GROUND;
#else
static const bridge_role_t LOCAL_ROLE = ROLE_GROUND;
static const bridge_role_t PEER_ROLE = ROLE_AIR;
#endif

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static esp_err_t parse_mac(const char *text, uint8_t out[ESP_NOW_ETH_ALEN])
{
    for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++) {
        int hi = hex_digit(text[i * 3]);
        int lo = hex_digit(text[i * 3 + 1]);
        if (hi < 0 || lo < 0) {
            return ESP_ERR_INVALID_ARG;
        }
        out[i] = (uint8_t)((hi << 4) | lo);
        if (i < ESP_NOW_ETH_ALEN - 1 && text[i * 3 + 2] != ':') {
            return ESP_ERR_INVALID_ARG;
        }
    }
    return text[17] == '\0' ? ESP_OK : ESP_ERR_INVALID_ARG;
}

#if CONFIG_BRIDGE_ENABLE_STATS
static void diag_write(const char *text)
{
    usb_serial_jtag_write_bytes(text, strlen(text), pdMS_TO_TICKS(50));
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(50));
}

static void diag_printf(const char *fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n > 0) {
        diag_write(line);
    }
}

static esp_err_t diag_init(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = {
            .tx_buffer_size = 4096,
            .rx_buffer_size = 4096,
        };
        return usb_serial_jtag_driver_install(&config);
    }
    return ESP_OK;
}
#else
static void diag_printf(const char *fmt, ...)
{
    (void)fmt;
}

static esp_err_t diag_init(void)
{
    return ESP_OK;
}
#endif

static uint16_t packet_crc(const bridge_packet_t *packet, size_t packet_len)
{
    bridge_packet_t tmp;
    memcpy(&tmp, packet, packet_len);
    tmp.crc = 0;
    return esp_crc16_le(UINT16_MAX, (const uint8_t *)&tmp, (uint32_t)packet_len);
}

static size_t packet_len_for_payload(size_t payload_len)
{
    return offsetof(bridge_packet_t, payload) + payload_len;
}

static void send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (tx_info != NULL && memcmp(tx_info->des_addr, s_peer_mac, ESP_NOW_ETH_ALEN) == 0) {
        s_last_send_status = status;
        xSemaphoreGive(s_send_done_sem);
    }
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (info == NULL || data == NULL || len < (int)offsetof(bridge_packet_t, payload)) {
        return;
    }

    if (memcmp(info->src_addr, s_peer_mac, ESP_NOW_ETH_ALEN) != 0) {
        s_rx_bad_peer++;
        return;
    }

    bridge_packet_t packet = { 0 };
    if ((size_t)len > sizeof(packet)) {
        s_rx_drops++;
        return;
    }
    memcpy(&packet, data, (size_t)len);

    const size_t expected_len = packet_len_for_payload(packet.len);
    if (packet.magic != BRIDGE_MAGIC || packet.version != BRIDGE_VERSION ||
        packet.role != PEER_ROLE || packet.flags != BRIDGE_FLAG_NONE ||
        packet.len > CONFIG_BRIDGE_PACKET_PAYLOAD_MAX || (size_t)len != expected_len) {
        s_rx_drops++;
        return;
    }

    if (packet_crc(&packet, expected_len) != packet.crc) {
        s_rx_bad_crc++;
        return;
    }

    if (s_have_last_rx_seq) {
        const uint16_t expected_seq = (uint16_t)(s_last_rx_seq + 1);
        if (packet.seq != expected_seq) {
            s_rx_seq_gaps++;
        }
    }
    s_last_rx_seq = packet.seq;
    s_have_last_rx_seq = true;

    bridge_item_t item = { .len = packet.len };
    memcpy(item.bytes, packet.payload, packet.len);
    if (xQueueSend(s_inbound_queue, &item, 0) == pdTRUE) {
        s_rx_packets++;
        s_espnow_rx_bytes += packet.len;
    } else {
        s_rx_drops++;
    }
}

static esp_err_t wifi_espnow_init(void)
{
    ESP_RETURN_ON_ERROR(parse_mac(CONFIG_BRIDGE_PEER_MAC, s_peer_mac), "bridge", "peer mac");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), "bridge", "nvs erase");
        nvs_err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(nvs_err, "bridge", "nvs");
    ESP_RETURN_ON_ERROR(esp_netif_init(), "bridge", "netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), "bridge", "event loop");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), "bridge", "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), "bridge", "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), "bridge", "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), "bridge", "wifi ps");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), "bridge", "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(CONFIG_BRIDGE_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE),
                        "bridge", "wifi channel");
    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, s_local_sta_mac), "bridge", "wifi mac");

    ESP_RETURN_ON_ERROR(esp_now_init(), "bridge", "esp-now init");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(send_cb), "bridge", "send cb");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(recv_cb), "bridge", "recv cb");

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    peer.channel = CONFIG_BRIDGE_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), "bridge", "peer");

#if CONFIG_BRIDGE_CONFIGURE_ESPNOW_RATE
    esp_now_rate_config_t rate_config = {
        .phymode = WIFI_PHY_MODE_11G,
        .rate = CONFIG_BRIDGE_ESPNOW_PHY_RATE,
        .ersu = false,
        .dcm = false,
    };
    ESP_RETURN_ON_ERROR(esp_now_set_peer_rate_config(s_peer_mac, &rate_config),
                        "bridge", "peer rate");
#endif

    return ESP_OK;
}

#if CONFIG_BRIDGE_ROLE_AIR
static esp_err_t local_io_init(void)
{
    const uart_config_t config = {
        .baud_rate = CONFIG_BRIDGE_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(EXTERNAL_UART, 8192, 8192, 0, NULL, 0),
                        "bridge", "uart driver");
    ESP_RETURN_ON_ERROR(uart_param_config(EXTERNAL_UART, &config), "bridge", "uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(EXTERNAL_UART,
                                     CONFIG_BRIDGE_UART_TX_GPIO,
                                     CONFIG_BRIDGE_UART_RX_GPIO,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        "bridge", "uart pins");
    ESP_RETURN_ON_ERROR(uart_flush(EXTERNAL_UART), "bridge", "uart flush");
    return ESP_OK;
}

static int local_read(uint8_t *buf, size_t max_len)
{
    return uart_read_bytes(EXTERNAL_UART, buf, max_len, pdMS_TO_TICKS(2));
}

static void local_write(const uint8_t *buf, size_t len)
{
    uart_write_bytes(EXTERNAL_UART, (const char *)buf, len);
}
#else
static esp_err_t local_io_init(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = {
            .tx_buffer_size = 4096,
            .rx_buffer_size = 4096,
        };
        ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&config), "bridge", "usb jtag");
    }
    return ESP_OK;
}

static int local_read(uint8_t *buf, size_t max_len)
{
    return usb_serial_jtag_read_bytes(buf, max_len, pdMS_TO_TICKS(2));
}

static void local_write(const uint8_t *buf, size_t len)
{
    size_t written_total = 0;
    while (written_total < len) {
        int written = usb_serial_jtag_write_bytes(buf + written_total,
                                                  len - written_total,
                                                  pdMS_TO_TICKS(20));
        if (written <= 0) {
            break;
        }
        written_total += (size_t)written;
    }
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(20));
}
#endif

static esp_err_t send_payload(const uint8_t *buf, size_t len)
{
    bridge_packet_t packet = {
        .magic = BRIDGE_MAGIC,
        .seq = s_tx_seq++,
        .len = (uint16_t)len,
        .version = BRIDGE_VERSION,
        .role = LOCAL_ROLE,
        .flags = BRIDGE_FLAG_NONE,
    };
    memcpy(packet.payload, buf, len);

    const size_t packet_len = packet_len_for_payload(len);
    packet.crc = packet_crc(&packet, packet_len);

    xSemaphoreTake(s_send_done_sem, 0);
    esp_err_t err = esp_now_send(s_peer_mac, (const uint8_t *)&packet, packet_len);
    if (err != ESP_OK) {
        s_send_fail++;
        return err;
    }

    if (xSemaphoreTake(s_send_done_sem, pdMS_TO_TICKS(CONFIG_BRIDGE_SEND_TIMEOUT_MS)) != pdTRUE) {
        s_send_timeout++;
        return ESP_ERR_TIMEOUT;
    }
    if (s_last_send_status != ESP_NOW_SEND_SUCCESS) {
        s_send_fail++;
        return ESP_FAIL;
    }

    s_send_ok++;
    s_espnow_tx_bytes += (uint32_t)len;
    return ESP_OK;
}

static void local_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[LOCAL_IO_BUFFER_SIZE];
    while (true) {
        int n = local_read(buf, sizeof(buf));
        if (n <= 0) {
            vTaskDelay(1);
            continue;
        }

        bridge_item_t item = { .len = (uint16_t)n };
        memcpy(item.bytes, buf, (size_t)n);
        s_local_rx_bytes += (uint32_t)n;
        if (xQueueSend(s_outbound_queue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            s_rx_drops++;
        }
    }
}

static void espnow_tx_task(void *arg)
{
    (void)arg;
    bridge_item_t item;
    while (true) {
        if (xQueueReceive(s_outbound_queue, &item, portMAX_DELAY) == pdTRUE) {
            (void)send_payload(item.bytes, item.len);
        }
    }
}

static void local_tx_task(void *arg)
{
    (void)arg;
    bridge_item_t item;
    while (true) {
        if (xQueueReceive(s_inbound_queue, &item, portMAX_DELAY) == pdTRUE) {
            local_write(item.bytes, item.len);
            s_local_tx_bytes += item.len;
        }
    }
}

#if CONFIG_BRIDGE_ENABLE_STATS
static void stats_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        diag_printf("bridge role=%u sta=%02x:%02x:%02x:%02x:%02x:%02x peer=%02x:%02x:%02x:%02x:%02x:%02x "
                    "local_rx=%lu local_tx=%lu esp_tx=%lu esp_rx=%lu send_ok=%lu send_fail=%lu "
                    "send_timeout=%lu rx_packets=%lu rx_drops=%lu rx_bad_crc=%lu rx_bad_peer=%lu rx_seq_gaps=%lu\n",
                    (unsigned)LOCAL_ROLE,
                    s_local_sta_mac[0], s_local_sta_mac[1], s_local_sta_mac[2],
                    s_local_sta_mac[3], s_local_sta_mac[4], s_local_sta_mac[5],
                    s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
                    s_peer_mac[3], s_peer_mac[4], s_peer_mac[5],
                    (unsigned long)s_local_rx_bytes,
                    (unsigned long)s_local_tx_bytes,
                    (unsigned long)s_espnow_tx_bytes,
                    (unsigned long)s_espnow_rx_bytes,
                    (unsigned long)s_send_ok,
                    (unsigned long)s_send_fail,
                    (unsigned long)s_send_timeout,
                    (unsigned long)s_rx_packets,
                    (unsigned long)s_rx_drops,
                    (unsigned long)s_rx_bad_crc,
                    (unsigned long)s_rx_bad_peer,
                    (unsigned long)s_rx_seq_gaps);
    }
}
#endif

#if CONFIG_BRIDGE_TEST_PATTERN
static void test_pattern_task(void *arg)
{
    (void)arg;
    uint32_t count = 0;
    char line[96];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int n = snprintf(line, sizeof(line), "[espnow-uart test] role=%u count=%lu\n",
                         (unsigned)LOCAL_ROLE, (unsigned long)count++);
        if (n > 0) {
            bridge_item_t item = { .len = (uint16_t)n };
            memcpy(item.bytes, line, (size_t)n);
            xQueueSend(s_outbound_queue, &item, 0);
        }
    }
}
#endif

void app_main(void)
{
    ESP_ERROR_CHECK(diag_init());
    diag_printf("bridge boot role=%u channel=%u peer=%s\n",
                (unsigned)LOCAL_ROLE, (unsigned)CONFIG_BRIDGE_WIFI_CHANNEL,
                CONFIG_BRIDGE_PEER_MAC);

    s_outbound_queue = xQueueCreate(CONFIG_BRIDGE_QUEUE_DEPTH, sizeof(bridge_item_t));
    s_inbound_queue = xQueueCreate(CONFIG_BRIDGE_QUEUE_DEPTH, sizeof(bridge_item_t));
    s_send_done_sem = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_outbound_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(s_inbound_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(s_send_done_sem == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    esp_err_t err = local_io_init();
    if (err != ESP_OK) {
        diag_printf("bridge local_io_init failed: %s\n", esp_err_to_name(err));
        ESP_ERROR_CHECK(err);
    }
    err = wifi_espnow_init();
    if (err != ESP_OK) {
        diag_printf("bridge wifi_espnow_init failed: %s\n", esp_err_to_name(err));
        ESP_ERROR_CHECK(err);
    }
    diag_printf("bridge ready role=%u sta=%02x:%02x:%02x:%02x:%02x:%02x peer=%02x:%02x:%02x:%02x:%02x:%02x\n",
                (unsigned)LOCAL_ROLE,
                s_local_sta_mac[0], s_local_sta_mac[1], s_local_sta_mac[2],
                s_local_sta_mac[3], s_local_sta_mac[4], s_local_sta_mac[5],
                s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
                s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);

#if !CONFIG_BRIDGE_TEST_PATTERN && !CONFIG_BRIDGE_ENABLE_STATS
    xTaskCreate(local_rx_task, "bridge_local_rx", CONFIG_BRIDGE_TASK_STACK, NULL, 10, NULL);
#endif
    xTaskCreate(espnow_tx_task, "bridge_esp_tx", CONFIG_BRIDGE_TASK_STACK, NULL, 9, NULL);
    xTaskCreate(local_tx_task, "bridge_local_tx", CONFIG_BRIDGE_TASK_STACK, NULL, 9, NULL);
#if CONFIG_BRIDGE_ENABLE_STATS
    xTaskCreate(stats_task, "bridge_stats", CONFIG_BRIDGE_TASK_STACK, NULL, 4, NULL);
#endif
#if CONFIG_BRIDGE_TEST_PATTERN
    xTaskCreate(test_pattern_task, "bridge_test", CONFIG_BRIDGE_TASK_STACK, NULL, 5, NULL);
#endif
}
