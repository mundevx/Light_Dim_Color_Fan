/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee touchlink light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_zb_light.h"
#include <esp_check.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "esp_zigbee_core.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_system.h"
#include "zboss_api.h"
#include "cli/zb_esp_cli.h"
#include "zb_config_platform.h"
#include "zb_vendor_default.h"
#include "driver/ledc.h"



#if ! defined ZB_ROUTER_ROLE
#error define ZB_ROUTER_ROLE to compile
#endif /* defined ZB_ROUTER_ROLE */


/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems
 *    integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and
 *    the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "switch_driver.h"
#include "esp_zigbee_trace.h"

/**
 * @brief:
 * This example code shows how to configure light switch with attribute as well as button switch handler.
 *
 * @note:
   Currently only support toggle switch functionality available
 *
 * @note:
 * For other possible switch functions (on/off,level up/down,step up/down). User need to implement and create them by themselves
 */

static QueueHandle_t gpio_evt_queue = NULL;
/* button function pair, should be defined in switch example source file */
static switch_func_pair_t *switch_func_pair;
/* call back function pointer */
static esp_switch_callback_t func_ptr;
/* which button is pressed */
static uint8_t switch_num;
static const char *TAG = "ESP_ZB_SWITCH";

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    xQueueSendFromISR(gpio_evt_queue, (switch_func_pair_t *)arg, NULL);
}

/**
 * @brief Enable GPIO (switchs refer to) isr
 *
 * @param enabled      enable isr if true.
 */
static void switch_driver_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < switch_num; ++i) {
        if (enabled) {
            gpio_intr_enable((switch_func_pair + i)->pin);
        } else {
            gpio_intr_disable((switch_func_pair + i)->pin);
        }
    }
}

/**
 * @brief Tasks for checking the button event and debounce the switch state
 *
 * @param arg      Unused value.
 */
static void switch_driver_button_detected(void *arg)
{
    gpio_num_t io_num = GPIO_NUM_NC;
    switch_func_pair_t button_func_pair;
    static switch_state_t switch_state = SWITCH_IDLE;
    bool evt_flag = false;

    for (;;) {
        /* check if there is any queue received, if yes read out the button_func_pair */
        if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY)) {
            io_num =  button_func_pair.pin;
            switch_driver_gpios_intr_enabled(false);
            evt_flag = true;
        }
        while (evt_flag) {
            bool value = gpio_get_level(io_num);
            switch (switch_state) {
            case SWITCH_IDLE:
                switch_state = (value == GPIO_INPUT_LEVEL_ON) ? SWITCH_PRESS_DETECTED : SWITCH_IDLE;
                break;
            case SWITCH_PRESS_DETECTED:
                switch_state = (value == GPIO_INPUT_LEVEL_ON) ? SWITCH_PRESS_DETECTED : SWITCH_RELEASE_DETECTED;
                break;
            case SWITCH_RELEASE_DETECTED:
                switch_state = SWITCH_IDLE;
                /* callback to button_handler */
                (*func_ptr)(&button_func_pair);
                break;
            default:
                break;
            }
            if (switch_state == SWITCH_IDLE) {
                switch_driver_gpios_intr_enabled(true);
                evt_flag = false;
                break;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

/**
 * @brief init GPIO configuration as well as isr
 *
 * @param button_func_pair      pointer of the button pair.
 * @param button_num            number of button pair.
 */
static bool switch_driver_gpio_init(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
    gpio_config_t io_conf = {};
    switch_func_pair = button_func_pair;
    switch_num = button_num;
    uint64_t pin_bit_mask = 0;

    /* set up button func pair pin mask */
    for (int i = 0; i < button_num; ++i) {
        pin_bit_mask |= (1ULL << (button_func_pair + i)->pin);
    }
    /* interrupt of falling edge */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    /* configure GPIO with the given settings */
    gpio_config(&io_conf);
    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(switch_func_pair_t));
    if ( gpio_evt_queue == 0) {
        ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    /* start gpio task */
    xTaskCreate(switch_driver_button_detected, "button_detected", 2048, NULL, 10, NULL);
    /* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    for (int i = 0; i < button_num; ++i) {
        gpio_isr_handler_add((button_func_pair + i)->pin, gpio_isr_handler, (void *) (button_func_pair + i));
    }
    return true;
}

//ggg
bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t cb)
{
    if (!switch_driver_gpio_init(button_func_pair, button_num)) {
        return false;
    }
    func_ptr = cb;
    return true;
}

/**
 * @brief init GPIO configuration as well as isr
 *
 * @param button_func_pair      pointer of the button pair.
 * @param button_num            number of button pair.
 */
static bool switch_driver_gpio_init_wo_isr_service(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
    gpio_config_t io_conf = {};
    switch_func_pair = button_func_pair;
    switch_num = button_num;
    uint64_t pin_bit_mask = 0;

    /* set up button func pair pin mask */
    for (int i = 0; i < button_num; ++i) {
        pin_bit_mask |= (1ULL << (button_func_pair + i)->pin);
    }
    /* interrupt of falling edge */
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    /* configure GPIO with the given settings */
    gpio_config(&io_conf);
    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(switch_func_pair_t));
    if ( gpio_evt_queue == 0) {
        ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    /* start gpio task */
    xTaskCreate(switch_driver_button_detected, "button_detected", 2048, NULL, 10, NULL);
    /* install gpio isr service */
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    for (int i = 0; i < button_num; ++i) {
        gpio_isr_handler_add((button_func_pair + i)->pin, gpio_isr_handler, (void *) (button_func_pair + i));
    }
    return true;
}

//ggg
bool switch_driver_init_wo_isr_service(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
    if (!switch_driver_gpio_init_wo_isr_service(button_func_pair, button_num)) {
        return false;
    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t endpoint_counts 			= 0;
static uint8_t gateway_type 			= 0;

#define GATEWAY_CHANNEL_SONOFF         	0x19   //0x14   //20
#define GATEWAY_CHANNEL_SONOFF_DONGLE  	0x0B   //11
#define GATEWAY_CHANNEL_TUYA           	0x0f   //15


#define GATEWAY_SONOFF                 	1
#define GATEWAY_TUYA                   	2


#define ON_OFF_LIGHT 					1
#define DIM_LIGHT 						2
#define DIM_COLOR_LIGHT 				3
#define FAN_CTRL        				4

#define USE_ZB_DEVICE_ID   				FAN_CTRL //ON_OFF_LIGHT

//static uint8_t channelx = 0;

#define TOUCHLINK_TARGET_TIMEOUT 		60

#define FAN_CLUSTER   ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL //ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL //ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL
//esp_zb_ieee_addr_t global_ieee_addr;

typedef struct light_bulb_info_s
{
  esp_zb_ieee_addr_t ieee_addr;
  uint16_t short_addr;
  uint8_t endpoint;
} light_bulb_info_t;

typedef struct light_control_device_ctx_t
{
    light_bulb_info_t on_off_light;  /* persistent, remote device struct for recording and managing node info */
    esp_zb_ieee_addr_t pending_dev_addr;  /* addr of device which is pending for discovery */
    bool light_state;
    uint8_t light_level;
    uint16_t light_color_x;
	uint16_t light_color_y;
} light_control_device_ctx_t;

light_control_device_ctx_t g_device_ctx[MAX_EP_SIZE];  /* light control device ifnfomation */

typedef struct light_bulb_device_params_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t  endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;
light_bulb_device_params_t light[MAX_EP_SIZE];// = (light_bulb_device_params_t *)malloc(sizeof(light_bulb_device_params_t));

typedef struct zdo_info_ctx_s {
    uint8_t endpoint;
    uint16_t short_addr;
} zdo_info_user_ctx_t;



#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC1_OUTPUT_IO         CONFIG_EXAMPLE_STRIP_LED_GPIO1 // Define the output GPIO
#define LEDC1_CHANNEL           LEDC_CHANNEL_0

#define LEDC2_OUTPUT_IO         CONFIG_EXAMPLE_STRIP_LED_GPIO2 // Define the output GPIO
#define LEDC2_CHANNEL           LEDC_CHANNEL_1

#define LEDC3_OUTPUT_IO         CONFIG_EXAMPLE_STRIP_LED_GPIO3 // Define the output GPIO
#define LEDC3_CHANNEL           LEDC_CHANNEL_2

#define LEDC4_OUTPUT_IO         CONFIG_EXAMPLE_STRIP_LED_GPIO4 // Define the output GPIO
#define LEDC4_CHANNEL           LEDC_CHANNEL_3

#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

uint16_t dim_levels[10] = {0,512,1024,1500,2048,3000,4095,5568,6800,8191};
uint8_t pwm_channels[4] = { LEDC1_CHANNEL,LEDC2_CHANNEL, LEDC3_CHANNEL, LEDC4_CHANNEL};
/* R, G, B of color x,y define table */
static uint16_t color_x_table[3] = {
    41942, 19660, 9830
};
static uint16_t color_y_table[3] = {
    21626, 38321, 3932
};


static void esp_zb_led_blink_task(void *pvParameters){
	light_driver_set_color_RGB(0xff, 0x00, 0x00);
	while(1){
		light_driver_set_power(true);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		light_driver_set_power(false);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static void esp_zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    /* By checking the button function pair to call different cmd send */
	uint8_t index = 1;
	if(button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH1 || button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH2 ||
			button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH3 || button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH4){
	    switch (button_func_pair->func) {
			case SWITCH_ONOFF_TOGGLE_CONTROL:
				if(button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH1){
					index = 0;
	    		}else if(button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH2){
	    			index = 1;
				}else if(button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH3){
					index = 2;
				}else if(button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH4){
					index = 3;
				}

				if(g_device_ctx[index].light_state) g_device_ctx[index].light_state = false;
				else g_device_ctx[index].light_state = true;
				//g_device_ctx[index].light_state = !g_device_ctx[index].light_state;
				gpio_set_level(gpio_led[index], g_device_ctx[index].light_state);
				ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command to address(0x%x) endpoint(%d))",
						g_device_ctx[index].on_off_light.short_addr, g_device_ctx[index].on_off_light.endpoint);
				ESP_EARLY_LOGI(TAG, "Value Sent : %d", g_device_ctx[index].light_state );
				//g_device_ctx[index].on_off_light.endpoint = ENDPOINTS_LIST[index];   //to be removed
				if(index < MAX_EP_SIZE){
					//set local attribute
// 			    	esp_zb_zcl_report_attr_cmd_t des;
// 			    	des.zcl_basic_cmd.dst_addr_u.addr_short=0;   // gateway
// 			    	des.zcl_basic_cmd.dst_endpoint = g_device_ctx[index].on_off_light.endpoint;
// 			    	des.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index];
// 			    	des.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
// 			    	des.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;//0xFFFE;
// 			    	des.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
// 			    	des.cluster_role=ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
// 			    	esp_zb_zcl_attr_t *value_r =
// 			    			esp_zb_zcl_get_attribute(g_device_ctx[index].on_off_light.endpoint,ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
// 			    			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
// 			    	*((uint32_t*) value_r->data_p)+=1;
// 			    	esp_zb_zcl_set_attribute_val(g_device_ctx[index].on_off_light.endpoint,ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
// 			    			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,(bool*)&light_state[index], false);
// //					 used for SONOFF gateway additionally
// 					if(channelx == GATEWAY_CHANNEL_SONOFF || channelx == GATEWAY_CHANNEL_SONOFF_DONGLE){
// 						esp_zb_zcl_report_attr_cmd_req(&des);
// 					}
					if(index < MAX_EP_SIZE){


						//set local attribute
//						esp_zb_zcl_status_t status1 = esp_zb_zcl_set_attribute_val(
//								ENDPOINTS_LIST[index],//g_device_ctx[index].on_off_light.endpoint,
//								ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
//								ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
//								ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
//								(uint8_t*)&g_device_ctx[index].light_state, //g_device_ctx[index].light_state,
//								false
//							);
						esp_zb_zcl_status_t status = 0;
						if(ENDPOINTS_TYPE[index] == FAN_LEVEL_ID){
							uint8_t val = 0;
							if(g_device_ctx[index].light_state) val = 3;

//							#if(FAN_CLUSTER == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL)
//								status = esp_zb_zcl_set_attribute_val(
//										ENDPOINTS_LIST[index],
//										ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
//										ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
//										0x04,
//										(uint8_t*)&val,
//										false
//									);
//							#elif(FAN_CLUSTER == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
								status = esp_zb_zcl_set_attribute_val(
										ENDPOINTS_LIST[index],
										ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
										ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
										ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
										(uint8_t*)&val,
										false
									);
//							#endif
						}else{
							status = esp_zb_zcl_set_attribute_val(
									ENDPOINTS_LIST[index],//ENDPOINTS_LIST[index],//g_device_ctx[index].on_off_light.endpoint,
									ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
									ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
									ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
									(uint8_t*)&g_device_ctx[index].light_state,
									false
								);
						}

						#if(USE_ZB_DEVICE_ID == DIM_LIGHT)
						esp_zb_zcl_status_t status11 = esp_zb_zcl_set_attribute_val(
								ENDPOINTS_LIST[index],//g_device_ctx[index].on_off_light.endpoint,
								ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
								ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
								ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
								(uint8_t*)&g_device_ctx[index].light_state,
								false
							);
						#elif(USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT)
						esp_zb_zcl_status_t status21 = esp_zb_zcl_set_attribute_val(
								ENDPOINTS_LIST[index],//g_device_ctx[index].on_off_light.endpoint,
								ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
								ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
								ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
								(uint8_t*)&g_device_ctx[index].light_state,
								false
							);
						esp_zb_zcl_status_t status22 = esp_zb_zcl_set_attribute_val(
								ENDPOINTS_LIST[index],//g_device_ctx[index].on_off_light.endpoint,
								ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
								ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
								ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID,
								(uint8_t*)&g_device_ctx[index].light_state,
								false
							);
						esp_zb_zcl_status_t status23 = esp_zb_zcl_set_attribute_val(
								ENDPOINTS_LIST[index],//g_device_ctx[index].on_off_light.endpoint,
								ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
								ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
								ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID,
								(uint8_t*)&g_device_ctx[index].light_state,
								false
							);
						#endif

						if (status == ESP_ZB_ZCL_STATUS_SUCCESS) {
							ESP_LOGI(TAG, "Set attribute successful. Sending config report...");
	                        // used for SONOFF gateway additionally
							//if(gateway_type == GATEWAY_SONOFF){  //|| channelx == GATEWAY_CHANNEL_SONOFF_DONGLE
//								esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
//								report_attr_cmd.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[index];
//								report_attr_cmd.zcl_basic_cmd.dst_addr_u.addr_short = g_device_ctx[0].on_off_light.short_addr;
//								report_attr_cmd.zcl_basic_cmd.dst_endpoint = g_device_ctx[index].on_off_light.endpoint;
//								report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
//								report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
//								report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
//								report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
//								esp_err_t error = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
//								if(error == ESP_OK) ESP_LOGI(TAG, "Report Attribute Successful...SRC_EP: %d", ENDPOINTS_LIST[index]);
//								else ESP_LOGI(TAG, "Report Attribute Failure...");
//                                ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command to address(0x%x) endpoint(%d))",
//						        g_device_ctx[0].on_off_light.short_addr, g_device_ctx[index].on_off_light.endpoint);
						    //}
						}else{
							ESP_LOGI(TAG, "Set attribute Failure.");
						}
					}else{
						ESP_EARLY_LOGI(TAG, "You Crosses Endpoints Limit %d", MAX_EP_SIZE);
					}
				}else{
					ESP_EARLY_LOGI(TAG, "You Crosses Endpoints Limit %d", MAX_EP_SIZE);
				}
				break;
			default:
				break;
	    }
	}else if(button_func_pair->pin == GPIO_INPUT_IO_TOGGLE_SWITCH){
		//esp_zb_zdo_device_leave_req();
		//free(esp_zb_led_blink_task);
		esp_zb_factory_reset(); //
		//esp_restart();
	}
}

static void ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    	ESP_LOGI(TAG, "*****************************ACTIVE ENDPOINTS LISTS***********************");
        ESP_LOGI(TAG, "Active endpoint response: status(%d) and endpoint count(%d)", zdo_status, ep_count);
        for (int i = 0; i < ep_count; i++) {
            ESP_LOGI(TAG, "Endpoint ID List: %d", ep_id_list[i]);
            g_device_ctx[i].on_off_light.endpoint = ep_id_list[i];
            //memcpy(&(g_device_ctx[i].on_off_light.ieee_addr), global_ieee_addr, sizeof(esp_zb_ieee_addr_t));
            g_device_ctx[i].on_off_light.short_addr = g_device_ctx[0].on_off_light.short_addr;
        }
    }
}

static void simple_desc_cb(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS){
        ESP_LOGI(TAG, "Simple desc resposne: status(%d), device_id(%d), app_version(%d), profile_id(0x%x), endpoint_ID(%d)", zdo_status,
                 simple_desc->app_device_id, simple_desc->app_device_version, simple_desc->app_profile_id, simple_desc->endpoint);
        for (int i = 0; i < (simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count); i++) {
            ESP_LOGI(TAG, "Cluster ID list: 0x%x", *(simple_desc->app_cluster_list + i));
        }
        /* get the node simple descriptor */
        esp_zb_zdo_simple_desc_req_param_t simple_desc_req;
        simple_desc_req.addr_of_interest = light[0].short_addr;
        simple_desc_req.endpoint = light[0].endpoint;
        esp_zb_zdo_simple_desc_req(&simple_desc_req, simple_desc_cb, NULL);
    }
}

static void bind_cbx(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Bound successfully!");
        if (user_ctx) {
        	light_bulb_device_params_t  *light = (light_bulb_device_params_t  *)user_ctx;
        	g_device_ctx[endpoint_counts].on_off_light.endpoint = light->endpoint;
        	g_device_ctx[endpoint_counts].on_off_light.short_addr = light->short_addr;
        	memcpy(&(g_device_ctx[endpoint_counts].on_off_light.ieee_addr), light->ieee_addr, sizeof(esp_zb_ieee_addr_t));
            ESP_LOGI(TAG, "The light originating from address(0x%x) on endpoint(%d)", light->short_addr, light->endpoint);
            endpoint_counts++;
            if(endpoint_counts >= MAX_EP_SIZE){ endpoint_counts = 0;
//				for(int i=0; i<MAX_EP_SIZE; i++) {
//					ESP_LOGI(TAG, "LIGHTEP: %d", g_device_ctx[i].on_off_light.endpoint);
//					ESP_LOGI(TAG, "SWITCHEP: %d", ENDPOINTS_LIST[i]);
//					ESP_LOGI(TAG, "SHORTADDR: %d", g_device_ctx[0].on_off_light.short_addr);
//					esp_zb_zcl_read_attr_cmd_t read_req;
//					uint16_t attributes[] = {ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID};
//					read_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;//
//					read_req.attr_number = sizeof(attributes) / sizeof(uint16_t);
//					read_req.attr_field = attributes;
//					read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
//					read_req.zcl_basic_cmd.dst_endpoint = g_device_ctx[i].on_off_light.endpoint;
//					read_req.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[i];
//					read_req.zcl_basic_cmd.dst_addr_u.addr_short = g_device_ctx[0].on_off_light.short_addr;
//					esp_zb_zcl_read_attr_cmd_req(&read_req);
//				}
//                ESP_LOGI(TAG, "Binding Complete!!");
//				for(int i=0; i<MAX_EP_SIZE; i++) {
//					esp_zb_zcl_config_report_cmd_t report_cmd;
//					bool report_change = 1;
//					report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = g_device_ctx[0].on_off_light.short_addr;
//					report_cmd.zcl_basic_cmd.dst_endpoint = g_device_ctx[i].on_off_light.endpoint;
//					report_cmd.zcl_basic_cmd.src_endpoint = ENDPOINTS_LIST[i];
//					report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
//					report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
//
//			        esp_zb_zcl_config_report_record_t records[] = {
//			            {ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, ESP_ZB_ZCL_ATTR_TYPE_BOOL, 0, 30, &report_change}};
//			        report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
//			        report_cmd.record_field = records;
//					esp_zb_zcl_config_report_cmd_req(&report_cmd);
//				}
            }
        }
    }
}

static void ieee_cbx(esp_zb_zdp_status_t zdo_status, esp_zb_ieee_addr_t ieee_addr, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {

		for(int i=0; i<MAX_EP_SIZE; i++) {
			esp_zb_zdo_bind_req_param_t bind_req;
			memcpy(&(bind_req.src_address), ieee_addr, sizeof(esp_zb_ieee_addr_t));
			esp_zb_ieee_address_by_short(light[i].short_addr, light[i].ieee_addr);
			esp_zb_get_long_address(bind_req.src_address);
			bind_req.src_endp = ENDPOINTS_LIST[i];

			bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
			memcpy(bind_req.dst_address_u.addr_long, ieee_addr, sizeof(esp_zb_ieee_addr_t));
			bind_req.dst_endp = g_device_ctx[i].on_off_light.endpoint;
			bind_req.req_dst_addr = esp_zb_get_short_address(); /* TODO: Send bind request to self */

		#if(USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT)
			bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL;
			ESP_LOGI(TAG, "Try to bind color control");
			esp_zb_zdo_device_bind_req(&bind_req, bind_cbx, NULL);
			bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
			ESP_LOGI(TAG, "Try to bind level control");
		#elif(USE_ZB_DEVICE_ID == DIM_LIGHT)
			bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
			ESP_LOGI(TAG, "Try to bind level control");
			esp_zb_zdo_device_bind_req(&bind_req, bind_cbx, NULL);
			bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
			ESP_LOGI(TAG, "Try to bind Light %d On/Off", light[i].endpoint);
        #elif(USE_ZB_DEVICE_ID == ON_OFF_LIGHT)
			bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
			ESP_LOGI(TAG, "Try to bind Light %d On/Off", light[i].endpoint);
        #elif(USE_ZB_DEVICE_ID == FAN_CTRL)
		    if(ENDPOINTS_TYPE[i] == FAN_LEVEL_ID){
//				#if(FAN_CLUSTER == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL)
//					bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
//				#else
					//bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
		    	bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
//				#endif

		    }else if(ENDPOINTS_TYPE[i] == ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID){
		    	bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
		    }
		#endif
			esp_zb_zdo_device_bind_req(&bind_req, bind_cbx, (void *)&light[i]);
		}
    }
}

static void user_find_cbx(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Found light");

        light[0].endpoint = endpoint;
        light[0].short_addr = addr;

        light[1].endpoint = HA_ESP_LIGHT_ENDPOINT2;
        light[1].short_addr = addr;

        light[2].endpoint = HA_ESP_LIGHT_ENDPOINT3;
        light[2].short_addr = addr;

        light[3].endpoint = HA_ESP_LIGHT_ENDPOINT4;
        light[3].short_addr = addr;

        ESP_LOGI(TAG, "*****************************FINDING ENDPOINTS***********************");
        endpoint_counts = 0;
        esp_zb_zdo_active_ep_req_param_t active_ep_req;
        active_ep_req.addr_of_interest = addr;
        esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb, NULL);

        /* get the node simple descriptor */
        esp_zb_zdo_simple_desc_req_param_t simple_desc_req;
        simple_desc_req.addr_of_interest = addr;
        simple_desc_req.endpoint = endpoint;
        esp_zb_zdo_simple_desc_req(&simple_desc_req, simple_desc_cb, NULL);

        /* get the light ieee address */
        esp_zb_zdo_ieee_addr_req_param_t ieee_req;
        ieee_req.addr_of_interest = addr;
        ieee_req.dst_nwk_addr = addr;
        ieee_req.request_type = 0;
        ieee_req.start_index = 0;
        esp_zb_zdo_ieee_addr_req(&ieee_req, ieee_cbx, NULL);

        ESP_LOGI(TAG, "Light Endpoint(%d)", endpoint);

//        zb_uint8_t ep_num = get_endpoint_by_cluster(0xEF00, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
//        ESP_LOGI(TAG, "Cluster Count: %d", ep_num);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}
//void find_light_bulb(uint16_t short_addr)
//{
//    esp_zb_zdo_match_desc_req_param_t  find_req;
//    find_req.addr_of_interest = short_addr;
//    find_req.dst_nwk_addr = short_addr;
//    /* Find the match on-off light device */
//    esp_zb_zdo_find_on_off_light(&find_req, user_find_cbx, NULL);
//    memset(&g_device_ctx[0].pending_dev_addr, 0, sizeof(esp_zb_ieee_addr_t));
//}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    //esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        } else {
            /* device auto start successfully and on a formed network */
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

            gateway_type = GATEWAY_TUYA;
            //esp_zb_zcl_get_attribute
            //channelx = esp_zb_get_current_channel();
            //ESP_LOGI(TAG, "Channel: 0x%x", channelx);
            esp_zb_zdo_match_desc_req_param_t find_req;
           // uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL};
            find_req.dst_nwk_addr = 0x0000;
            find_req.addr_of_interest = 0x0000;
            find_req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
           //  find_req.num_in_clusters = 5;
           //  find_req.num_out_clusters = 0;
           //  find_req.cluster_list = cluster_list;
//			#if(USE_ZB_DEVICE_ID == FAN_CTRL)
//				uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF};   //ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL
//				find_req.cluster_list = cluster_list;
//				find_req.num_in_clusters = 5;
//				find_req.num_out_clusters = 0;
//				esp_zb_zdo_match_cluster (&find_req, user_find_cbx, NULL);
//			#else
				esp_zb_zdo_find_on_off_light(&find_req, user_find_cbx, NULL);
//			#endif
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        if(esp_zb_get_short_address() != 0xffff){
        	ESP_LOGI(TAG, "Device Already Joined Network");
        }
    	break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
//        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
//        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
//            ESP_LOGI(TAG, "Reset device");
//        }
    	esp_zb_zdo_signal_leave_params_t *leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
		if (leave_params && leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
			ESP_LOGI(TAG, "Device Left");                                        // erase previous network information.
			//esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING); // steering a new network.
			while(1){
				xTaskCreate(esp_zb_led_blink_task, "Zigbee_led_blink", 4096, NULL, 6, NULL);
			}
		}
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
//        if (err_status == ESP_OK) {
//            esp_zb_zdo_active_ep_req_param_t active_ep_req;
//            active_ep_req.addr_of_interest = light[0].short_addr;
//            esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb, NULL);
//        }
        break;
    }

}


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t* message) {

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);
    uint8_t index = 0;
    uint8_t cluster_type = 0;

    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT1) {
    	index = 0;
    	ESP_LOGI(TAG, "Received ENDPOINT1");
        switch (message->info.cluster) {
			case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
				cluster_type = ON_OFF_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
					g_device_ctx[0].light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : g_device_ctx[0].light_state;
					ESP_LOGI(TAG, "Light1 sets to %s", g_device_ctx[0].light_state ? "On" : "Off");
					//light_driver_set_power(light_state);
				} else {
					ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
				cluster_type = DIM_COLOR_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[0].light_color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[0].light_color_x;
					g_device_ctx[0].light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light1 color x changes to 0x%x", g_device_ctx[0].light_color_x);
				} else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
						   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[0].light_color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[0].light_color_y;
					g_device_ctx[0].light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light1 color y changes to 0x%x", g_device_ctx[0].light_color_y);
				} else {
					ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}

				break;
			case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
				cluster_type = DIM_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
					g_device_ctx[0].light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : g_device_ctx[0].light_level;
					//light_driver_set_level((uint8_t)light_level);
					ESP_LOGI(TAG, "Light1 level changes to %d", g_device_ctx[0].light_level);
				} else {
					ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			default:
				ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }else if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT2) {
    	index = 1;
    	ESP_LOGI(TAG, "Received ENDPOINT2");
        switch (message->info.cluster) {
			case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
				cluster_type = ON_OFF_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
					g_device_ctx[1].light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : g_device_ctx[1].light_state;
					ESP_LOGI(TAG, "Light2 sets to %s", g_device_ctx[1].light_state ? "On" : "Off");
				} else {
					ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
				cluster_type = DIM_COLOR_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[1].light_color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[1].light_color_x;
					g_device_ctx[1].light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light2 color x changes to 0x%x", g_device_ctx[1].light_color_x);
				} else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
						   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[1].light_color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[1].light_color_y;
					g_device_ctx[1].light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light2 color y changes to 0x%x", g_device_ctx[1].light_color_y);
				} else {
					ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}

				break;
			case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
				cluster_type = DIM_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
					g_device_ctx[1].light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : g_device_ctx[1].light_level;
					//light_driver_set_level((uint8_t)light_level);
					ESP_LOGI(TAG, "Light2 level changes to %d", g_device_ctx[1].light_level);
				} else {
					ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			default:
				ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }else if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT3) {
    	index = 2;
    	ESP_LOGI(TAG, "Received ENDPOINT3");
        switch (message->info.cluster) {
			case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
				cluster_type = ON_OFF_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
					g_device_ctx[2].light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : g_device_ctx[2].light_state;
					ESP_LOGI(TAG, "Light3 sets to %s", g_device_ctx[2].light_state ? "On" : "Off");
					//light_driver_set_power(light_state);


				} else {
					ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
				cluster_type = DIM_COLOR_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[2].light_color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[2].light_color_x;
					g_device_ctx[2].light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light3 color x changes to 0x%x", g_device_ctx[2].light_color_x);
				} else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
						   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[2].light_color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[2].light_color_y;
					g_device_ctx[2].light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light3 color y changes to 0x%x", g_device_ctx[2].light_color_y);
				} else {
					ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}

				break;
			case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
				cluster_type = DIM_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
					g_device_ctx[2].light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : g_device_ctx[2].light_level;
					//light_driver_set_level((uint8_t)light_level);
					ESP_LOGI(TAG, "Light3 level changes to %d", g_device_ctx[2].light_level);
				} else {
					ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			default:
				ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }else if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT4) {
    	index = 3;
        switch (message->info.cluster) {
			case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
				cluster_type = ON_OFF_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
					g_device_ctx[3].light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : g_device_ctx[3].light_state;
					ESP_LOGI(TAG, "Light4 sets to %s", g_device_ctx[3].light_state ? "On" : "Off");
					//light_driver_set_power(light_state);
					ESP_LOGI(TAG, "Received ENDPOINT4");

				} else {
					ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
				cluster_type = DIM_COLOR_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[3].light_color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[3].light_color_x;
					g_device_ctx[3].light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light4 color x changes to 0x%x", g_device_ctx[3].light_color_x);
				} else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
						   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
					g_device_ctx[3].light_color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : g_device_ctx[3].light_color_y;
					g_device_ctx[3].light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
																		  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
										 ->data_p;
					ESP_LOGI(TAG, "Light4 color y changes to 0x%x", g_device_ctx[3].light_color_y);
				} else {
					ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}

				break;
			case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
				cluster_type = DIM_LIGHT;
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
					g_device_ctx[3].light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : g_device_ctx[3].light_level;
					//light_driver_set_level((uint8_t)light_level);
					ESP_LOGI(TAG, "Light4 level changes to %d", g_device_ctx[3].light_level);
				} else {
					ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
				}
				break;
			default:
				ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    if(cluster_type == ON_OFF_LIGHT){
    	gpio_set_level(gpio_led[index], g_device_ctx[index].light_state);
    }else if(cluster_type == DIM_COLOR_LIGHT){
    	//light_driver_set_color_xy(g_device_ctx[index].light_color_x, g_device_ctx[index].light_color_y);
    	uint16_t dim_val = map(g_device_ctx[index].light_level, 0, 255, 0, 8191);
		// Set duty to 50%
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], dim_val));
		// Update duty to apply the new value
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
    }else if(cluster_type == DIM_LIGHT){
    	//light_driver_set_level((uint8_t)g_device_ctx[index].light_level);

    	uint16_t dim_val = map(g_device_ctx[index].light_level, 0, 255, 0, 8191);
		// Set duty to 50%
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[index], dim_val));
		// Update duty to apply the new value
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[index]));
    }
    return ESP_OK;
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Reveived report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->src_address.u.short_addr,
             message->src_endpoint, message->dst_endpoint, message->cluster);
    ESP_LOGI(TAG, "Received report information: attribute(0x%x), type(0x%x), value(%d)\n", message->attribute.id, message->attribute.data.type,
             message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0);

//    uint8_t value = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
//    if(message->src_endpoint == HA_ESP_LIGHT_ENDPOINT1){
//    	light_state[0] = value;
//    	gpio_set_level(gpio_led[0], value);
//    }else if(message->src_endpoint == HA_ESP_LIGHT_ENDPOINT2){
//    	light_state[1] = value;
//		gpio_set_level(gpio_led[1], value);
//    }else if(message->src_endpoint == HA_ESP_LIGHT_ENDPOINT3){
//    	light_state[2] = value;
//		gpio_set_level(gpio_led[2], value);
//    }else if(message->src_endpoint == HA_ESP_LIGHT_ENDPOINT4){
//    	light_state[3] = value;
//		gpio_set_level(gpio_led[3], value);
//    }
    return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), attribute(0x%x)", message->info.status, message->info.cluster,
                 variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Received action: %d", callback_id);
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ESP_LOGI(TAG, "Received action: ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID");
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t*)message);
        break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

uint8_t lsl_attr_0 = 0;

#define LSL_CLUSTER_ID 0xEF00
const uint16_t attr_id = 0;
const uint8_t attr_type = ESP_ZB_ZCL_ATTR_TYPE_U32;
const uint8_t attr_access = ESP_ZB_ZCL_ATTR_MANUF_SPEC | ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE;
void *attr_value = &lsl_attr_0;

esp_zb_cluster_list_t* server_cluster(uint8_t index){
    uint8_t test_attr=0;
    // jfjss8rl
//    uint8_t uint8_stack_version  = 2;
//    uint8_t uint8_zcl_version_id = 3;
//    uint8_t uint8_zpower_src_id  = 1;
//    uint8_t uint8_hw_version_id  = 1;
//    uint16_t uint8_group_id      = 1;
//    uint16_t uint8_scene_id      = 0;
    //4831b7fffec06c5d
    #if(USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT)
    	//char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'm', 'e', 'x', 'i', 's', 'f', 'i', 'k'};
	    //char modelid[] = {6,'T', 'S', '1', '0', '0', '1'};
//        char manufname[] = {16, '_', 'T', 'Y', 'Z', 'B', '0', '1', '_', 'b', 'n', 'g', 'w', 'd', 'j', 's', 'r'};
//		char modelid[] = {6,'T', 'S', '1', '1', '0', '1'};
//
		char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
		char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

	#elif(USE_ZB_DEVICE_ID == DIM_LIGHT)
#if (MAX_EP_SIZE == 1)
		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '9', 'v', '8', '6', 'h', 'x', '9', 'o'};  //1-gang ok - works with both
		char modelid[] = {6,'T', 'S', '1', '0', '0', '1'};
#elif (MAX_EP_SIZE == 2)
//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '9', '2', 'c', 'h', 's', 'k', 'y', '7'};  //2-gang dimmer-works only with tuya
//		char modelid[] = {6,'T', 'S', '1', '1', '0', 'F'};

		char manufname[] = {16, '_', 'T', 'Y', 'Z', 'B', '0', '1', '_', 'v', '8', 'g', 't', 'i', 'a', 'e', 'd'};  //2-gang dimmer not work with tuya
		char modelid[] = {6,'T', 'S', '1', '1', '0', 'F'};
#elif (MAX_EP_SIZE == 3)  //pending
		char manufname[] = {16, '_', 'T', 'Y', 'Z', 'B', '0', '1', '_', 'v', '8', 'g', 't', 'i', 'a', 'e', 'd'};  //2-gang dimmer not work with tuya
		char modelid[] = {6,'T', 'S', '1', '1', '0', 'F'};
#elif (MAX_EP_SIZE == 4)  //pending  _TZ3000_xabckq1v
		//char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'x', 'a', 'b', 'c', 'k', 'q', '1', 'v'};
//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '4', 'f', 'j', 'i', 'w', 'w', 'e', 'b'};
//		char modelid[] = {6,'T', 'S', '0', '0', '4', 'F'};
//		char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
//		char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '4', 'f', 'j', 'i', 'w', 'w', 'e', 'b'};
		char modelid[] = {6,'T', 'S', '0', '6', '0', '1'};
#else
		char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
		char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
#endif
//	    char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
//	    char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', '9', 'v', '8', '6', 'h', 'x', '9', 'o'};  //1-gang dimmer
//		char modelid[] = {6,'T', 'S', '1', '1', '0', '1'};
//		char manufname[] = {16, '_', 'T', 'Z', '3', '2', '1', '0', '_', 'k', '4', 'm', 's', 'u', 'v', 'g', '6'};
//		char modelid[] = {6,'T', 'S', '1', '1', '0', 'E'};

//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'x', 'a', 'b', 'c', 'k', 'q', '1', 'v'};  //not working
//		char modelid[] = {6,'T', 'S', '0', '0', '4', 'F'};

		//_TZ3210_k1msuvg6     TS110E

//        char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', '1', 'a', 'g', 'w', 'n', 'e', 'm', 's'}; //not working
//		char modelid[] = {6,'T', 'S', '0', '6', '0', '1'};
//        char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '4', '_', 'z', 'e', 'n', 'j', '4', 'l', 'x', 'v'}; // not working
//		char modelid[] = {6,'T', 'S', '0', '6', '0', '1'};

//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'c', 'q', 'u', 'a', 'v', 'b', 'q', '5'};
//		char modelid[] = {8,'N', 'U', 'O', 'S', '4', 'C', 'H', 'D'};

	#elif(USE_ZB_DEVICE_ID == ON_OFF_LIGHT)
		//jfjss8rl

		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'j', 'f', 'j', 's', 's', '8', 'r', 'l'};
		char modelid[] = {6,'S', 'M', '0', '0', '1', '4'};

//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'm', 'e', 'x', 'i', 's', 'f', 'i', 'k'};
//		char modelid[] = {6,'S', 'M', '0', '0', '1', '4'};

//	    char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
//	    char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
        // to be check
//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'w', 'k', 'r', '3', 'j', 'q', 'm', 'r'};
//		char modelid[] = {6,'T', 'S', '0', '0', '0', '4'};
        //to be tested
//		char manufname[] = {16, '_', 'T', 'Z', '3', '0', '0', '0', '_', 'q', 'e', 'w', 'o', '8', 'd', 'l', 'z'};
//		char modelid[] = {6,'T', 'S', '0', '0', '1', '4'};
//      char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'm', 'e', 'x', 'i', 's', 'f', 'i', 'k'};
//		char modelid[] = {6,'T', 'S', '0', '6', '0', '1'};
    #elif(USE_ZB_DEVICE_ID == FAN_CTRL)
//		char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
//		char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

//		char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 't', '6', 'g', 'q', '6', 'n', 'j', 'u'};
//		char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'r', '3', '2', 'c', 't', 'e', 'z', 'x'};
//		char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'd', 'z', 'u', 'q', 'w', 's', 'y', 'g'};  //thermostat



		//char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'f', 'v', 'l', 'd', 'k', 'u', '9', 'h'};
		//char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'h', 'm', 'q', 'z', 'f', 'q', 'm', 'l'};   //_TZE200_hmqzfqm
//		char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'r', '3', '2', 'c', 't', 'e', 'z', 'x'};   //_TZE200_r32ctezx
//		char modelid[] = {6,'T', 'S', '0', '6', '0', '1'};

//		char manufname[] = {16, '_', 'T', 'Z', '3', '2', '1', '0', '_', 'l', 'z', 'q', 'q', '3', 'u', '4', 'r'};
//		char modelid[] = {6,'T', 'S', '0', '5', '0', '1'};


		//pk4hfmbb  -- custom nuos
		//b3qqkxck

		//gkxy2tha
		//char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'b', '3', 'q', 'q', 'k', 'x', 'c', 'k'};
		////char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', 'g', 'k', 'x', 'y', '2', 't', 'h', 'a'};
		//char modelid[] = {6,'T', 'S', '0', '6', '0', '1'};



		//8iwretuu
		char manufname[] = {16, '_', 'T', 'Z', 'E', '2', '0', '0', '_', '8', 'i', 'w', 'r', 'e', 't', 'u', 'u'};
		char modelid[] = {6,'T', 'S', '0', '6', '0', '1'};
	#endif
    test_attr  = 0;
    uint8_t zcl_ver = 3;
    uint8_t pwr_src = 1;
    uint8_t stack_ver = 0;
    uint8_t hw_ver = 1;
    /* basic cluster create with fully customized */
    ESP_LOGI(TAG, "*****************************BASIC CLUSTER***********************");
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &pwr_src);
    esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_ver);
//
//    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_ver);
//    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_ver);
//    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DEVICE_ENABLED_ID, &zcl_ver);

    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);
    ESP_LOGI(TAG, "*****************************IDENTIFY CLUSTER***********************");
    /* identify cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);
    ESP_LOGI(TAG, "*****************************POWER CLUSTER***********************");
    esp_zb_attribute_list_t *esp_zb_power_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
//    esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &test_attr1);
//    esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &test_attr1);
//    esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, &bat_quantity);
    ESP_LOGI(TAG, "*****************************GROUP CLUSTER***********************");
    /* group cluster create with fully customized */
    uint16_t group_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &group_id);
    /* scenes cluster create with standard cluster + customized */
    uint8_t scene_count = 0;//MAX_EP_SIZE;
    uint8_t scene_id = 0;//index+1;
    ESP_LOGI(TAG, "*****************************SCENE CLUSTER***********************");
    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(NULL);
//    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_CURRENT_GROUP_ID, &group_id);
	esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_CURRENT_SCENE_ID, &scene_id);
//	esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);
    //esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_SCENES);
//    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_SCENE_COUNT_ID, &scene_count);
//    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_CURRENT_SCENE_ID, &test_attr);
//    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);
//#if(USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT)

//	esp_zb_level_cluster_cfg_t level_cfg;
//	level_cfg.current_level = 50;
//	esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_level_cluster_create(&level_cfg);

//#elif(USE_ZB_DEVICE_ID == DIM_LIGHT)
    uint8_t current_level = 50;
    uint8_t min_level = 0;
    uint8_t max_level = 100;
    uint8_t on_level = 1;
	esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
	esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &current_level);
	esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MIN_LEVEL_ID, &min_level);
	esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_MAX_LEVEL_ID, &max_level);
//	esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_ON_LEVEL_ID, &on_level);
////	esp_zb_level_cluster_cfg_t level_cfg;
////	level_cfg.current_level = 50;
////	esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_level_cluster_create(&level_cfg);

	esp_zb_attribute_list_t *esp_zb_thermostat_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);


    /* on-off cluster create with standard cluster config*/
    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    #if(USE_ZB_DEVICE_ID == FAN_CTRL)
    	esp_zb_attribute_list_t *esp_zb_fan_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL);
    	esp_zb_fan_control_cluster_add_attr(esp_zb_fan_cluster, ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID, &test_attr);
    	esp_zb_fan_control_cluster_add_attr(esp_zb_fan_cluster, ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID, &test_attr);
//    	esp_zb_fan_control_cluster_cfg_t  fan_cfg;
//    	fan_cfg.fan_mode = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_ON;//ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_DEFAULT_VALUE;
//    	fan_cfg.fan_mode_sequence = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_DEFAULT_VALUE;
//    	esp_zb_attribute_list_t *esp_zb_fan_cluster = esp_zb_fan_control_cluster_create(&fan_cfg);

    	//esp_zb_attribute_list_t *esp_zb_time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    	//esp_zb_attribute_list_t *esp_zb_tuya_cluster = esp_zb_zcl_attr_list_create(0xEF00);  //Tuya Cluster
//    	esp_zb_attribute_list_t *esp_zb_tuya_cluster1 = esp_zb_zcl_attr_list_create(0xFF66);

		esp_zb_attribute_list_t *esp_zb_tuya_cluster = esp_zb_zcl_attr_list_create(LSL_CLUSTER_ID);
		esp_zb_cluster_add_attr(esp_zb_tuya_cluster, LSL_CLUSTER_ID, attr_id, attr_type, attr_access, attr_value);
    #endif
    	//////////////////////////////////////////////////
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    /* create cluster lists for this endpoint */
//    if(index == 0){
		esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
		/* update basic cluster in the existed cluster list */
		//esp_zb_cluster_list_update_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
		esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
		esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
		esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list, esp_zb_scenes_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
		//esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
//
//    esp_zb_cluster_list_add_time_cluster(esp_zb_cluster_list, esp_zb_time_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

#if(USE_ZB_DEVICE_ID == ON_OFF_LIGHT)
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#elif(USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT)
    /*On off dimmable colour switch*/
	esp_zb_color_cluster_cfg_t colour_cfg;
	esp_zb_attribute_list_t *esp_zb_colour_cluster = esp_zb_color_control_cluster_create(&colour_cfg);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	if(esp_zb_cluster_list_add_color_control_cluster(esp_zb_cluster_list, esp_zb_colour_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE) == ESP_ERR_INVALID_ARG)
	{
		ESP_LOGI(TAG, "esp_zb_cluster_list_add_color_control_cluster ESP_ERR_INVALID_ARG");
	}
#elif(USE_ZB_DEVICE_ID == DIM_LIGHT)
	esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#elif(USE_ZB_DEVICE_ID == FAN_CTRL)

    if(ENDPOINTS_TYPE[index] == FAN_LEVEL_ID){
    	ESP_LOGI(TAG, "Setting Fan Cluster");
//#if(FAN_CLUSTER == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL)
//    	esp_zb_cluster_list_add_fan_control_cluster(esp_zb_cluster_list, esp_zb_fan_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
//#else
      esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
//#endif

    }else if(ENDPOINTS_TYPE[index] == ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID){
    	ESP_LOGI(TAG, "Setting Light Cluster");
	  esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	  //esp_zb_cluster_list_add_level_cluster(esp_zb_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }
#endif
    return esp_zb_cluster_list;
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack with Zigbee coordinator config */
	#if CONFIG_ESP_ZB_TRACE_ENABLE
	  //esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_INFO, ESP_ZB_TRACE_SUBSYSTEM_COMMON | ESP_ZB_TRACE_SUBSYSTEM_APP | ESP_ZB_TRACE_SUBSYSTEM_ZCL);
	esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_DEBUG, ESP_ZB_TRACE_SUBSYSTEM_APP | ESP_ZB_TRACE_SUBSYSTEM_ZCL);
	#endif
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    ////////////////////////////////////////////////////////////////////////////////
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
	ESP_LOGI(TAG, "*******************************START*******************************");
    for(int i=0; i<MAX_EP_SIZE; i++){
#if(USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT)
       esp_zb_ep_list_add_ep(esp_zb_ep_list,
    		   server_cluster(i),
    		   ENDPOINTS_LIST[i],
    		   ESP_ZB_AF_HA_PROFILE_ID,
			   ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID);
#elif(USE_ZB_DEVICE_ID == DIM_LIGHT)
       esp_zb_ep_list_add_ep(esp_zb_ep_list,
    		   server_cluster(i),
    		   ENDPOINTS_LIST[i],
    		   ESP_ZB_AF_HA_PROFILE_ID,
			   ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID);//ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID); //ESP_ZB_HA_DIMMER_SWITCH_DEVICE_ID
#elif(USE_ZB_DEVICE_ID == ON_OFF_LIGHT)
       esp_zb_ep_list_add_ep(esp_zb_ep_list,
    		   server_cluster(i),
    		   ENDPOINTS_LIST[i],
    		   ESP_ZB_AF_HA_PROFILE_ID,
			   ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);
#elif(USE_ZB_DEVICE_ID == FAN_CTRL)
           esp_zb_ep_list_add_ep(esp_zb_ep_list,
        		   server_cluster(i),
				   ENDPOINTS_LIST[i],
        		   ESP_ZB_AF_HA_PROFILE_ID,
				   ENDPOINTS_TYPE[i] ); //fan  ESP_ZB_HA_THERMOSTAT_DEVICE_ID  ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID, ESP_ZB_HA_SMART_PLUG_DEVICE_ID
#endif
       ESP_LOGI(TAG, "ENDPOINTS_LIST[%d]= %d", i, ENDPOINTS_LIST[i]);
    }

#if(USE_ZB_DEVICE_ID == FAN_CTRL)
//           esp_zb_ep_list_add_ep(esp_zb_ep_list,
//        		   server_cluster(0),
//				   ENDPOINTS_LIST[0],
//        		   ESP_ZB_AF_HA_PROFILE_ID,
//				   ENDPOINTS_TYPE[0] ); //fan  ESP_ZB_HA_THERMOSTAT_DEVICE_ID  ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID, ESP_ZB_HA_SMART_PLUG_DEVICE_ID
//

//           esp_zb_ep_list_add_ep(esp_zb_ep_list,
//				   server_cluster(1),
//				   ENDPOINTS_LIST[1],
//				   ESP_ZB_AF_HA_PROFILE_ID,
//				   ENDPOINTS_TYPE[1] );


//           esp_zb_ep_list_add_ep(esp_zb_ep_list,
//				   server_cluster(2),
//				   ENDPOINTS_LIST[2],
//				   ESP_ZB_AF_HA_PROFILE_ID,
//				   ENDPOINTS_TYPE[2] );

//           esp_zb_ep_list_add_ep(esp_zb_ep_list,
//				   server_cluster(3),
//				   ENDPOINTS_LIST[3],
//				   ESP_ZB_AF_HA_PROFILE_ID,
//				   ENDPOINTS_TYPE[3] );
#endif

    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_nvram_erase_at_start(true);
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

//#if((USE_ZB_DEVICE_ID == DIM_LIGHT) || (USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT))
static void example_ledc_init(void)
{
	for(int i=0; i< MAX_EP_SIZE; i++){
		// Prepare and then apply the LEDC PWM timer configuration
		ledc_timer_config_t ledc_timer = {
			.speed_mode       = LEDC_MODE,
			.timer_num        = LEDC_TIMER,
			.duty_resolution  = LEDC_DUTY_RES,
			.freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
			.clk_cfg          = LEDC_AUTO_CLK
		};
		ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

		// Prepare and then apply the LEDC PWM channel configuration
		ledc_channel_config_t ledc_channel = {
			.speed_mode     = LEDC_MODE,
			.channel        = pwm_channels[i],
			.timer_sel      = LEDC_TIMER,
			.intr_type      = LEDC_INTR_DISABLE,
			.gpio_num       = gpio_led[i],
			.duty           = 0, // Set duty to 0%
			.hpoint         = 0
		};
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

void setLedPwm(){
	for(int j=9; j>=0; j--){
		for(int i=0; i< MAX_EP_SIZE; i++){
			// Set duty to 50%
			ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channels[i], dim_levels[j]));
			// Update duty to apply the new value
			ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channels[i]));
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
//#endif
static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(CONFIG_EXAMPLE_STRIP_LED_GPIO1);
    gpio_reset_pin(CONFIG_EXAMPLE_STRIP_LED_GPIO2);
    gpio_reset_pin(CONFIG_EXAMPLE_STRIP_LED_GPIO3);
    gpio_reset_pin(CONFIG_EXAMPLE_STRIP_LED_GPIO4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CONFIG_EXAMPLE_STRIP_LED_GPIO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(CONFIG_EXAMPLE_STRIP_LED_GPIO2, GPIO_MODE_OUTPUT);
    gpio_set_direction(CONFIG_EXAMPLE_STRIP_LED_GPIO3, GPIO_MODE_OUTPUT);
    gpio_set_direction(CONFIG_EXAMPLE_STRIP_LED_GPIO4, GPIO_MODE_OUTPUT);

    gpio_set_level(CONFIG_EXAMPLE_STRIP_LED_GPIO1, 0);
    gpio_set_level(CONFIG_EXAMPLE_STRIP_LED_GPIO2, 0);
    gpio_set_level(CONFIG_EXAMPLE_STRIP_LED_GPIO3, 0);
    gpio_set_level(CONFIG_EXAMPLE_STRIP_LED_GPIO4, 0);
    // Set the LEDC peripheral configuration
//#if((USE_ZB_DEVICE_ID == DIM_LIGHT) || (USE_ZB_DEVICE_ID == DIM_COLOR_LIGHT))
	example_ledc_init();
//#endif
	//setLedPwm();
}


void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* Load Zigbee platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    /* Hardware related and device init */
    light_driver_init(LIGHT_DEFAULT_OFF);
    switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_zb_buttons_handler);
    switch_driver_init_wo_isr_service(button_func_pair1, PAIR_SIZE(button_func_pair1));
    switch_driver_init_wo_isr_service(button_func_pair2, PAIR_SIZE(button_func_pair2));
    switch_driver_init_wo_isr_service(button_func_pair3, PAIR_SIZE(button_func_pair3));
    switch_driver_init_wo_isr_service(button_func_pair4, PAIR_SIZE(button_func_pair4));
    configure_led();
    //MAX_EP_SIZE = sizeof(ENDPOINTS_LIST);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
