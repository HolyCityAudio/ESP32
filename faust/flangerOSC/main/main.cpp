/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
// #include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include "AC101.h"
#include "flangerOSC.h"

#define DELAYMIN 0.005
#define DELAYMAX 1.0
#define DELAYFACTOR 1.1
#define LFORATEMIN 0.02
#define LFORATEFACTOR 1.1
#define LFORATEMAX 6.0
#define LFOWIDTHMIN 0.02
#define LFOWIDTHFACTOR 1.1
#define LFOWIDTHMAX 1.0

#define BUTTONSUPPORT 1

#if BUTTONSUPPORT
extern "C" {
#include "button.h"
}

class Buttonmenu {
	public:
		int gpioCodeUp;
		int pioCodeDown;
		enum mode { intType, floatType };
		float * parameter;
		float lowerLimit;
		float upperLimit;
		float deltaFactor;
		class Buttonmenu * next;
		
		int add(int, int, mode, float *, float, float, float);
		int processKey(button_event_t);
};
#endif

extern "C" {
    void app_main(void);
}

void udp_server_init(char * deviceName, MapUI * pUI);
void initialise_wifi(uint8_t *, uint8_t *);
void wait_for_ip();

// see also wifi.cpp, this is half implemented
uint8_t wifi_ssid [] = "your-ssid";
uint8_t wifi_pass [] = "your-password";

int SR = 32000;
int BS = 128;
//  BS = 256 seems to be about the maximum you can set it.
//  maybe 384.  I'm setting it higher to try to avoid WDT CPU0 Idle errors
//  when I add more blocks to the DSP file
//  BS = 512 causes stack overflow and reboot
flangerOSC flangerOSC(SR,BS); 	


void app_main(void)
{
	float echoDelayLevel = DELAYMIN;
	float rate = 1.0;
	float width = 0.25;
    AC101 AC101;

#if BUTTONSUPPORT
	button_event_t ev;
	QueueHandle_t button_events = button_init(PIN_BIT(5) | PIN_BIT(13) | PIN_BIT(18)  | PIN_BIT(19) | PIN_BIT(23) | PIN_BIT(36));
#endif
	
	// try to get the wireless SSID and password from NVRAM
	esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        char ssid[65]; 		// 
		char password[33];
		int32_t restart_counter = 0;
		size_t ssid_size = sizeof(ssid);
		size_t password_size = sizeof(password);
		
        err = nvs_get_str(my_handle, "ssid", ssid, &ssid_size);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("NVRAM: ssid = %s\n", ssid);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
				printf("Setting ssid to poobert.");
				err = nvs_set_str(my_handle, "ssid", "poobert");
				printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
				
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
		
		err = nvs_get_str(my_handle, "password", password, &password_size);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("NVRAM: password = %s\n", password);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
				printf("Setting password to beep-boop.");
				err = nvs_set_str(my_handle, "password", "beep-boop");
				printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
				
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
		
		err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("NVRAM: restart_counter = %d\n", restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        printf("Updating restart counter in NVS ... ");
        restart_counter++;
        err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }
	
    AC101.begin();
	AC101.SetVolumeHeadphone(63);
	printf("ssid %s\n", wifi_ssid);	
    initialise_wifi(wifi_ssid, wifi_pass);
    wait_for_ip();
	
	MapUI* gooey = flangerOSC.fUI;
				
	udp_server_init("flanger/", gooey);

    flangerOSC.start();
	
    while (true) {
#if BUTTONSUPPORT
		if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
//			ESP_LOGI("Queue rx", "pin %d", ev.pin);
			if ((ev.pin == 36) && (ev.event == BUTTON_DOWN)) {
				if (echoDelayLevel > DELAYMIN) 
				{
					echoDelayLevel /= DELAYFACTOR;
				}
				else 
				{
					echoDelayLevel = 0;
				}
				flangerOSC.setParamValue("EchoLvl", echoDelayLevel);		
				ESP_LOGI("EchoDelay", "Down-> %f", echoDelayLevel);
			}			
			if ((ev.pin == 13) && (ev.event == BUTTON_DOWN)) {
				if (echoDelayLevel < DELAYMIN) 
				{
					echoDelayLevel = DELAYMIN;
				}
				if (echoDelayLevel < DELAYMAX) 
				{
					echoDelayLevel = std::min(DELAYMAX, echoDelayLevel * DELAYFACTOR);
				}
				else
				{
					echoDelayLevel = DELAYMAX;
				}
				flangerOSC.setParamValue("EchoLvl", echoDelayLevel);		
				ESP_LOGI("EchoDelay", "Up-> %f", echoDelayLevel);
			}
			if ((ev.pin == 5) && (ev.event == BUTTON_DOWN)) {
				if (width < LFOWIDTHMAX) 
				{
					width = width * LFOWIDTHFACTOR;
				}
				else 
				{
					width = LFOWIDTHMAX;
				}
				if (width < LFOWIDTHMIN)
				{
					width = LFOWIDTHMIN;
				}
				flangerOSC.setParamValue("Width", width);	
				ESP_LOGI("LFO Width", "Up=>%f", width);
			}
			if ((ev.pin == 18) && (ev.event == BUTTON_DOWN)) {
				if (width > LFOWIDTHMIN) 
				{
					width = width/LFOWIDTHFACTOR;
				}
				else 
				{
					width = 0.0;
				}
				flangerOSC.setParamValue("Width", width);	
				ESP_LOGI("LFO Width", "Down=>%f", width);
			}			
			if ((ev.pin == 19) && (ev.event == BUTTON_DOWN)) {
				if (rate > LFORATEMIN) 
				{
					rate = rate/LFORATEFACTOR;
				}
				else 
				{
					rate = 0.0;
				}
				flangerOSC.setParamValue("Rate", rate);	
				ESP_LOGI("LFO Rate", "Down->%f", rate);
			}
			if ((ev.pin == 23) && (ev.event == BUTTON_DOWN)) {
				if (rate <  LFORATEMIN) 
				{
					rate = LFORATEMIN;
				}
				if (rate <  LFORATEMAX) 
				{
					rate = rate * LFORATEFACTOR;
				}
				flangerOSC.setParamValue("Rate", rate);	
				ESP_LOGI("LFO Rate", "Up->%f", rate);
			}		
		}
	#endif
    }
}
