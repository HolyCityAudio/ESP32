/* stereoFlanger main

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define DELAYMIN 64
#define DELAYMAX 864
#define LFORATEMIN 0.02
#define LFORATEFACTOR 1.2
#define LFORATEMAX 8.0
#define LFOWIDTHMIN 0.02
#define LFOWIDTHFACTOR 1.2
#define LFOWIDTHMAX 1.0

extern "C" {
#include "button.h"
}

#include "AC101.h"
#include "basicFlanger.h"

extern "C" {
    void app_main(void);
    void initialise_wifi(void);
    QueueHandle_t * button_init(unsigned long long );
}

void app_main(void)
{
    int SR = 32000;
//  BS = 256 seems to be about the maximum you can set it.
//  maybe 384.  I'm setting it higher to try to avoid WDT CPU0 Idle errors
//  when I add more blocks to the DSP file
//  BS = 512 causes stack overflow and reboot
    int BS = 128;
 
	int delay = DELAYMIN;
	float rate = 1.0;
	float width = 0.25;

    AC101 AC101;
//    initialise_wifi();
    AC101.begin();
    AC101.SetVolumeHeadphone(63);
	button_event_t ev;
	QueueHandle_t button_events = button_init(PIN_BIT(5) | PIN_BIT(13) | PIN_BIT(18)  | PIN_BIT(19) | PIN_BIT(23) | PIN_BIT(36));

    basicFlanger basicFlanger(SR,BS);  
    basicFlanger.start();
								        
    while (true) {
		if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
//			ESP_LOGI("Queue rx", "pin %d", ev.pin);
			if ((ev.pin == 36) && (ev.event == BUTTON_DOWN)) {
				if (delay > DELAYMIN) 
				{
					delay--;
				}
				basicFlanger.setParamValue("Delay", delay);		
				ESP_LOGI("Flanger delay", "Down-> %d", delay);
			}			
			if ((ev.pin == 13) && (ev.event == BUTTON_DOWN)) {
				if (delay < DELAYMAX) 
				{
					delay++;
				}
				basicFlanger.setParamValue("Delay", delay);		
				ESP_LOGI("Flanger delay", "Up-> %d", delay);
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
				basicFlanger.setParamValue("Width", width);	
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
				basicFlanger.setParamValue("Width", width);	
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
				basicFlanger.setParamValue("Rate", rate);	
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
				basicFlanger.setParamValue("Rate", rate);	
				ESP_LOGI("LFO Rate", "Up->%f", rate);
			}		
		}
    }
}
