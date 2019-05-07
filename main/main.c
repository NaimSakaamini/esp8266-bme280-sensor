#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/adc.h"

#include "bme.h"

#define LOG_TAG "MAIN"

#define ESPNOW_CHANNEL CONFIG_ESPNOW_CHANNEL
#define ESPNOW_STATION_MAC CONFIG_ESPNOW_STATION_MAC
#define ESPNOW_PMK CONFIG_ESPNOW_PMK
#define ESPNOW_LMK CONFIG_ESPNOW_LMK
#define SLEEP_TIME CONFIG_SLEEP_TIME

#define MIN_VCC 2300
#define MAX_VCC 3000

typedef struct
{
    uint32_t pressure;
    int16_t temperature;
    uint16_t humidity;
    uint8_t battery;
}__attribute__((packed)) espnow_sensor_data_t;

static esp_now_peer_info_t station_peer;

static SemaphoreHandle_t espnow_send_lock;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    xSemaphoreGive(espnow_send_lock);
}

static void espnow_init()
{
    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
    ;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, 0));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));

    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));

    station_peer.channel = ESPNOW_CHANNEL;
    station_peer.ifidx = ESP_IF_WIFI_STA;
    station_peer.encrypt = true;
    memcpy(station_peer.lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
    const char *mac = ESPNOW_STATION_MAC;
    for (uint8_t i = 0; i < ESP_NOW_ETH_ALEN; i++) {
        station_peer.peer_addr[i] = (uint8_t) strtol(mac + (3 * i), (char **) NULL, 16);
    }
    ESP_LOGD(LOG_TAG, "Station MAC %02x:%02x:%02x:%02x:%02x:%02x", station_peer.peer_addr[0], station_peer.peer_addr[1],
            station_peer.peer_addr[2], station_peer.peer_addr[3], station_peer.peer_addr[4], station_peer.peer_addr[5]);

    ESP_ERROR_CHECK(esp_now_add_peer(&station_peer));
}

static void espnow_deinit()
{
    esp_now_deinit();
    esp_wifi_stop();
    esp_wifi_deinit();
}

static espnow_sensor_data_t fetch_data()
{
    espnow_sensor_data_t ret;
    bme_data_t data = bme_read();

    uint16_t vcc;
    adc_read(&vcc);

    ret.pressure = data.pressure;
    ret.temperature = data.temperature;
    ret.humidity = data.humidity;
    ret.battery = ((vcc > MAX_VCC ? MAX_VCC : vcc) - MIN_VCC) / (float) (MAX_VCC - MIN_VCC) * 100;

    ESP_LOGD(LOG_TAG, "Battery voltage %d mV", vcc);
    ESP_LOGD(LOG_TAG, "Battery capacity %d %%", ret.battery);
    return ret;
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    adc_config_t adc_config;
    adc_config.mode = ADC_READ_VDD_MODE;
    adc_config.clk_div = 8;
    ESP_ERROR_CHECK(adc_init(&adc_config));

    bme_init();
    espnow_init();

    espnow_send_lock = xSemaphoreCreateBinary();

    espnow_sensor_data_t espnow_data = fetch_data();
    uint8_t *bs = (uint8_t *) malloc(sizeof(espnow_sensor_data_t));
    memcpy(bs, &espnow_data, sizeof(espnow_sensor_data_t));
    esp_now_send(station_peer.peer_addr, bs, sizeof(espnow_sensor_data_t));

    xSemaphoreTake(espnow_send_lock, portMAX_DELAY);
    vSemaphoreDelete(espnow_send_lock);

    espnow_deinit();

    esp_deep_sleep(SLEEP_TIME * 1000000);
}
