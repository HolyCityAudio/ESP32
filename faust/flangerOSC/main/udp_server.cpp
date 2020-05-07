/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sys/param.h>
// #include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// #include "audio_pipeline.h"
// #include "i2s_stream.h"
// #include "board.h"
// #include "Midpoint.h"
#include "faust/gui/MapUI.h"


#include "oscpkt.hh"

#include <vector>

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

#define CONFIG_EXAMPLE_IPV4 true

#define CONFIG_EXAMPLE_PORT 9000
#define PORT CONFIG_EXAMPLE_PORT

using namespace oscpkt;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

MapUI * fUI;
char deviceString[64];
int deviceStringLength;

static const char *TAG = "udp_server";

static void udp_server_task(void *pvParameters)
{
    std::vector<char> rx_buffer;
    char addr_str[128];
	char message[64];
	char oscCommand[64];
    int addr_family;
    int ip_protocol;
	
	strcpy(oscCommand, "poobert");

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket binded");

        PacketReader pr;
// this is where you hook to the Faust program????

        while (1) {

            //ESP_LOGI(TAG, "Waiting for data");

            rx_buffer.resize(1024*32); //initial buffersize. 
            //normal size is 100k, but this creates overflow for the esp32
            
            struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(sourceAddr);

            //recieve the data
            int len = recvfrom(sock, &rx_buffer[0], (int)rx_buffer.size(), 0, (struct sockaddr *)&sourceAddr, &socklen);
            
            //resize and swap before reading the data
            rx_buffer.resize(len);
            std::vector<char> tmp(rx_buffer); 
            tmp.swap(rx_buffer);

            // Error occured during receiving
            if (len < 0) {
                ESP_LOGI(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {

                pr.init((rx_buffer.empty() ? 0 : &rx_buffer[0]), rx_buffer.size());
                oscpkt::Message *msg;

                if(!pr.isOk()){
                    ESP_LOGI(TAG, "ospkt error: %i", pr.getErr());
                }
                while (pr.isOk() && (msg = pr.popMessage()) != 0) {
                    float farg;
                    std::string address = msg->addressPattern();
                    address = address.substr(1, address.size() - 1);
                    msg->arg().popFloat(farg);
//                    ESP_LOGI(TAG, "message address: %s %s %d", address.c_str(), deviceString, deviceStringLength);
//                    ESP_LOGI(TAG, "message from: %f", farg);
// here's where we send parameter updates to the Faust object 
                    strcpy(message,address.c_str());
					if(!(strncmp(message, deviceString, deviceStringLength)))
					{
						int commandLength = 1 + strlen(message) - deviceStringLength;
//						ESP_LOGI(TAG, "commandLength: %d", commandLength);
						memcpy((void *)oscCommand, (const void *) message + deviceStringLength, commandLength);
						ESP_LOGI(TAG, "set oscCommand: /%s%s to %f", deviceString, oscCommand, farg);
						fUI->setParamValue(oscCommand, farg);					
					}	
                }
            }
			vTaskDelay(10/portTICK_PERIOD_MS);
        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
		vTaskDelay(10/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// void udp_server_init(string deviceName)
void udp_server_init(char * deviceName, MapUI * pUI)
{
	fUI = pUI;
	strcpy(deviceString, deviceName);
	deviceStringLength = strlen(deviceString);
    ESP_LOGI(TAG, "deviceString %s", (char *) deviceString);
 
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    
}
