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

#define ECHOFBMIN 0.02
#define ECHOFBMAX 1.0
#define ECHOFBFACTOR 1.1

#define ECHOLPFMAX 10000.0
#define ECHOLPFMIN 1000
#define ECHOLPFFACTOR 1.1

#define ECHOTIMEMAX 2.5
#define ECHOTIMEMIN 0.10
#define ECHOTIMEFACTOR 1.1

extern "C" {
#include "button.h"
}

#include "AC101.h"
#include "basicEcho.h"

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
 
	float echoFeedback = 0.0;
	float echoTime = 0.15;
	float echoLPF = 2500.0;

    AC101 AC101;
//    initialise_wifi();
    AC101.begin();
    AC101.SetVolumeHeadphone(63);
	button_event_t ev;
	QueueHandle_t button_events = button_init(PIN_BIT(5) | PIN_BIT(13) | PIN_BIT(18)  | PIN_BIT(19) | PIN_BIT(23) | PIN_BIT(36));

    basicEcho basicEcho(SR,BS);  
    basicEcho.start();
								        
    while (true) {
		if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
//			ESP_LOGI("Queue rx", "pin %d", ev.pin);
			if ((ev.pin == 13) && (ev.event == BUTTON_DOWN)) {
				if (echoFeedback < ECHOFBMAX) 
				{
					echoFeedback = echoFeedback * ECHOFBFACTOR;
				}
				else 
				{
					echoFeedback = ECHOFBMAX;
				}
				if (echoFeedback < ECHOFBMIN)
				{
					echoFeedback = ECHOFBMIN;
				}
				basicEcho.setParamValue("echoFeedback", echoFeedback);		
				ESP_LOGI("echoFeedback", "Up-> %f", echoFeedback);
			}			
			if ((ev.pin == 36) && (ev.event == BUTTON_DOWN)) {
				if (echoFeedback > ECHOFBMIN) 
				{
					echoFeedback = echoFeedback/ECHOFBFACTOR;
				}
				else 
				{
					echoFeedback = 0.0;
				}
				basicEcho.setParamValue("echoFeedback", echoFeedback);		
				ESP_LOGI("echoFeedback", "Down-> %f", echoFeedback);
			}

			if ((ev.pin == 18) && (ev.event == BUTTON_DOWN)) {
				if (echoLPF > ECHOLPFMIN) 
				{
					echoLPF = echoLPF/ECHOLPFFACTOR;
				}
				else 
				{
					echoLPF = ECHOLPFMIN;
				}
				basicEcho.setParamValue("echoLPF", echoLPF);	
				ESP_LOGI("echoLPF", "Down=>%f", echoLPF);
			}	
			if ((ev.pin == 5) && (ev.event == BUTTON_DOWN)) {
				if (echoLPF < ECHOLPFMAX) 
				{
					echoLPF = echoLPF * ECHOLPFFACTOR;
				}
				else 
				{
					echoLPF = ECHOLPFMAX;
				}
				if (echoLPF < ECHOLPFMIN)
				{
					echoLPF = ECHOLPFMIN;
				}
				basicEcho.setParamValue("echoLPF", echoLPF);	
				ESP_LOGI("echoLPF", "Up=>%f", echoLPF);	
			}
			if ((ev.pin == 19) && (ev.event == BUTTON_DOWN)) {
				if (echoTime > ECHOTIMEMIN) 
				{
					echoTime = echoTime/ECHOTIMEFACTOR;
				}
				else 
				{
					echoTime = ECHOTIMEMIN;
				}
				basicEcho.setParamValue("echoTime", echoTime);	
				ESP_LOGI("echoTime", "Down->%f", echoTime);
			}
			if ((ev.pin == 23) && (ev.event == BUTTON_DOWN)) {
				if (echoTime <  ECHOTIMEMIN) 
				{
					echoTime = ECHOTIMEMIN;
				}
				if (echoTime <  ECHOTIMEMAX) 
				{
					echoTime = echoTime * ECHOTIMEFACTOR;
				}
				basicEcho.setParamValue("echoTime", echoTime);	
				ESP_LOGI("echoTime", "Up->%f", echoTime);
			}		
		}
    }
}
