#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define BRIDGE_MAGIC 0x56555831u
#define BRIDGE_VERSION 1u
#define ESPNOW_PAYLOAD_MAX 1400
#define UART_CHUNK_MAX 512
#define QUEUE_DEPTH 32
#define EXTERNAL_UART UART_NUM_1

typedef enum {
    ROLE_GROUND = 1,
    ROLE_AIR = 2,
} bridge_role_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t seq;
    uint16_t len;
    uint8_t version;
    uint8_t role;
    uint8_t flags;
    uint8_t reserved;
    uint8_t payload[ESPNOW_PAYLOAD_MAX];
} bridge_packet_t;

typedef struct {
    uint16_t len;
    uint8_t bytes[ESPNOW_PAYLOAD_MAX];
} rx_item_t;

static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

static QueueHandle_t s_rx_queue;
static volatile uint32_t s_send_ok;
static volatile uint32_t s_send_fail;
static volatile uint32_t s_rx_packets;
static volatile uint32_t s_rx_drops;
#if CONFIG_BRIDGE_ROLE_AIR
static uint16_t s_tx_seq;
#endif

#if CONFIG_BRIDGE_ROLE_AIR
static const bridge_role_t LOCAL_ROLE = ROLE_AIR;
static const bridge_role_t PEER_ROLE = ROLE_GROUND;
#else
static const bridge_role_t PEER_ROLE = ROLE_AIR;
#endif

#if CONFIG_BRIDGE_ENABLE_STATS || !CONFIG_BRIDGE_ROLE_AIR
static void usb_output_init(void)
{
    if (usb_serial_jtag_is_driver_installed()) {
        return;
    }

    usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
}
#endif

static void send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_send_ok++;
    } else {
        s_send_fail++;
    }
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;

    if (len < (int)(sizeof(bridge_packet_t) - ESPNOW_PAYLOAD_MAX)) {
        return;
    }

    const bridge_packet_t *packet = (const bridge_packet_t *)data;
    const int header_len = sizeof(bridge_packet_t) - ESPNOW_PAYLOAD_MAX;
    if (packet->magic != BRIDGE_MAGIC || packet->version != BRIDGE_VERSION ||
        packet->role != PEER_ROLE || packet->len > ESPNOW_PAYLOAD_MAX ||
        len != header_len + packet->len) {
        return;
    }

    rx_item_t item = { .len = packet->len };
    memcpy(item.bytes, packet->payload, packet->len);
    if (xQueueSend(s_rx_queue, &item, 0) == pdTRUE) {
        s_rx_packets++;
    } else {
        s_rx_drops++;
    }
}

static esp_err_t wifi_espnow_init(void)
{
    ESP_RETURN_ON_ERROR(nvs_flash_init(), "bridge", "nvs");
    ESP_RETURN_ON_ERROR(esp_netif_init(), "bridge", "netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), "bridge", "event loop");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), "bridge", "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), "bridge", "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), "bridge", "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), "bridge", "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(CONFIG_BRIDGE_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE),
                        "bridge", "wifi channel");

    ESP_RETURN_ON_ERROR(esp_now_init(), "bridge", "esp-now init");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(send_cb), "bridge", "send cb");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(recv_cb), "bridge", "recv cb");

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    peer.channel = CONFIG_BRIDGE_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), "bridge", "broadcast peer");
    return ESP_OK;
}

#if CONFIG_BRIDGE_ROLE_AIR && !CONFIG_BRIDGE_TEST_PATTERN
static int source_read(uint8_t *buf, size_t max_len)
{
    return uart_read_bytes(EXTERNAL_UART, buf, max_len, pdMS_TO_TICKS(1));
}
#endif

#if CONFIG_BRIDGE_ROLE_AIR
static void send_payload(const uint8_t *buf, size_t len)
{
    bridge_packet_t packet = {
        .magic = BRIDGE_MAGIC,
        .version = BRIDGE_VERSION,
        .role = LOCAL_ROLE,
    };

    if (len > ESPNOW_PAYLOAD_MAX) {
        len = ESPNOW_PAYLOAD_MAX;
    }

    packet.seq = s_tx_seq++;
    packet.len = (uint16_t)len;
    memcpy(packet.payload, buf, len);
    const size_t header_len = sizeof(packet) - ESPNOW_PAYLOAD_MAX;
    if (esp_now_send(BROADCAST_MAC, (const uint8_t *)&packet, header_len + len) != ESP_OK) {
        s_send_fail++;
    }
}
#endif

static void sink_write(const uint8_t *buf, size_t len)
{
#if CONFIG_BRIDGE_ROLE_AIR
    uart_write_bytes(EXTERNAL_UART, (const char *)buf, len);
#else
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
#endif
}

#if CONFIG_BRIDGE_ENABLE_STATS
static void diagnostic_write(const uint8_t *buf, size_t len)
{
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    write(STDOUT_FILENO, buf, len);
}
#endif

#if CONFIG_BRIDGE_ROLE_AIR && !CONFIG_BRIDGE_TEST_PATTERN
static esp_err_t external_uart_init(void)
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
    return ESP_OK;
}
#endif

#if CONFIG_BRIDGE_ROLE_AIR && !CONFIG_BRIDGE_TEST_PATTERN
static void tx_task(void *arg)
{
    (void)arg;
    uint8_t buf[UART_CHUNK_MAX];
    while (true) {
        int n = source_read(buf, sizeof(buf));
        if (n < 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        send_payload(buf, (size_t)n);
    }
}
#endif

#if CONFIG_BRIDGE_ROLE_AIR && CONFIG_BRIDGE_TEST_PATTERN
static void test_pattern_task(void *arg)
{
    (void)arg;
    uint32_t count = 0;
    char line[80];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int n = snprintf(line, sizeof(line), "[espnow-uart test] air_count=%lu\n", (unsigned long)count++);
        if (n > 0) {
            send_payload((const uint8_t *)line, (size_t)n);
        }
    }
}
#endif

static void rx_task(void *arg)
{
    (void)arg;
    rx_item_t item;
    while (true) {
        if (xQueueReceive(s_rx_queue, &item, portMAX_DELAY) == pdTRUE) {
            sink_write(item.bytes, item.len);
        }
    }
}

#if CONFIG_BRIDGE_ENABLE_STATS
static void stats_task(void *arg)
{
    (void)arg;
    char line[160];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
#if CONFIG_BRIDGE_ROLE_AIR
        const char *role = "air";
#else
        const char *role = "ground";
#endif
        int n = snprintf(line,
                         sizeof(line),
                         "\n[espnow-uart %s] send_ok=%lu send_fail=%lu rx=%lu rx_drops=%lu\n",
                         role,
                         (unsigned long)s_send_ok,
                         (unsigned long)s_send_fail,
                         (unsigned long)s_rx_packets,
                         (unsigned long)s_rx_drops);
        if (n > 0) {
            diagnostic_write((const uint8_t *)line, (size_t)n);
        }
    }
}
#endif

void app_main(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

#if CONFIG_BRIDGE_ENABLE_STATS || !CONFIG_BRIDGE_ROLE_AIR
    usb_output_init();
#endif

#if CONFIG_BRIDGE_ENABLE_STATS
    diagnostic_write((const uint8_t *)"\nESP-NOW UART bridge boot\n", 26);
#endif

    s_rx_queue = xQueueCreate(QUEUE_DEPTH, sizeof(rx_item_t));
#if CONFIG_BRIDGE_ENABLE_STATS
    esp_err_t init_err = wifi_espnow_init();
    if (init_err != ESP_OK) {
        char err_line[96];
        int err_len = snprintf(err_line,
                               sizeof(err_line),
                               "ESP-NOW UART bridge init failed: %s\n",
                               esp_err_to_name(init_err));
        if (err_len > 0) {
            diagnostic_write((const uint8_t *)err_line, (size_t)err_len);
        }
        xTaskCreate(stats_task, "bridge_stats", 4096, NULL, 1, NULL);
        return;
    }
#else
    ESP_ERROR_CHECK(wifi_espnow_init());
#endif
#if CONFIG_BRIDGE_ROLE_AIR && !CONFIG_BRIDGE_TEST_PATTERN
    ESP_ERROR_CHECK(external_uart_init());
#endif

#if CONFIG_BRIDGE_ENABLE_STATS
    char line[192];
    int n = snprintf(line,
                     sizeof(line),
                     "\nESP-NOW UART bridge role=%s mac=%02x:%02x:%02x:%02x:%02x:%02x channel=%d baud=%d\n",
#if CONFIG_BRIDGE_ROLE_AIR
            "air",
#else
            "ground",
#endif
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     CONFIG_BRIDGE_WIFI_CHANNEL,
                     CONFIG_BRIDGE_UART_BAUD);
    if (n > 0) {
        diagnostic_write((const uint8_t *)line, (size_t)n);
    }
#else
    (void)mac;
#endif

#if CONFIG_BRIDGE_ROLE_AIR && !CONFIG_BRIDGE_TEST_PATTERN
    xTaskCreatePinnedToCore(tx_task, "bridge_tx", 4096, NULL, 10, NULL, 0);
#endif
    xTaskCreatePinnedToCore(rx_task, "bridge_rx", 4096, NULL, 11, NULL, 0);
#if CONFIG_BRIDGE_ROLE_AIR && CONFIG_BRIDGE_TEST_PATTERN
    xTaskCreate(test_pattern_task, "bridge_test", 4096, NULL, 1, NULL);
#endif
#if CONFIG_BRIDGE_ENABLE_STATS
    xTaskCreate(stats_task, "bridge_stats", 4096, NULL, 1, NULL);
#endif
}
