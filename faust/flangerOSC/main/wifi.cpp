#include <sys/param.h>
#include <cstring>
// #include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"


/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// TODO: debug, I am trying to figure out how to restore the wifi creedntial from NVRAM, so far
// it is not working so I've just hardwired the credentials at the top of this file.

#define EXAMPLE_ESP_WIFI_SSID "your-ssid"
#define EXAMPLE_ESP_WIFI_PASS "your-password"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
static const char *TAG = "wifi";
const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    }
    case SYSTEM_EVENT_STA_CONNECTED:
    {
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    }
    case SYSTEM_EVENT_STA_GOT_IP:
    {
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    }
    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        break;
    }
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
    {
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    }
    default:
        break;
    }
    return ESP_OK;
}

void initialise_wifi(uint8_t * user_ssid, uint8_t * user_password)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

	#if 1  
	// debug, I am trying to figure out how to restore the wifi creedntial from NVRAM, so far
	// it is not working so I've just hardwired the credentials at the top of this file.
    wifi_config_t wifi_config = {
        .sta = {
            {.ssid = EXAMPLE_ESP_WIFI_SSID},
            {.password = EXAMPLE_ESP_WIFI_PASS}
        },
    };
	#else
	
	wifi_config_t wifi_config;
	memcpy(wifi_config.sta.ssid,user_ssid, 64);
	memcpy(wifi_config.sta.password,user_password, 32);
	#endif
	
    ESP_LOGI(TAG, "Setting WiFi configuration SSID to %s/%s...", wifi_config.sta.ssid, wifi_config.sta.password);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}
