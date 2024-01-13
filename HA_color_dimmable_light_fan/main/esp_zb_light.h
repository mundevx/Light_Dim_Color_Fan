/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
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

#include "esp_zigbee_core.h"
#include "light_driver.h"
#include "zboss_api.h"
#include "zboss_api_tl.h"
#include "switch_driver.h"

/* Zigbee configuration */
#define MAX_CHILDREN                    	10          /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE 			false             /* enable the install code policy for security */
#define HA_ESP_LIGHT_ENDPOINT1          	1          /* esp light bulb device endpoint */
#define HA_ESP_LIGHT_ENDPOINT2          	101          /* esp light bulb device endpoint */
#define HA_ESP_LIGHT_ENDPOINT3          	103          /* esp light bulb device endpoint */
#define HA_ESP_LIGHT_ENDPOINT4          	102          /* esp light bulb device endpoint */
#define HA_ESP_LIGHT_ENDPOINT5          	5
#define HA_ESP_LIGHT_ENDPOINT6          	6

#define HA_COLOR_DIMMABLE_LIGHT_ENDPOINT1  	1
#define HA_COLOR_DIMMABLE_LIGHT_ENDPOINT2  	2
#define HA_COLOR_DIMMABLE_LIGHT_ENDPOINT3  	3
#define HA_COLOR_DIMMABLE_LIGHT_ENDPOINT4  	4

//#define ESP_ZB_TOUCHLINK_CHANNEL        	11
//#define ESP_ZB_TOUCHLINK_CHANNEL_MASK 	(1l << ESP_ZB_TOUCHLINK_CHANNEL)    /* Zigbee touchlink channel mask use in the example */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     	ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define TOUCHLINK_TARGET_TIMEOUT 			60                 /* The timeout for the Touchlink target, measured in seconds */

#define MAX_EP_SIZE    						3
//size_t EP_SIZE  = MAX_EP_SIZE;
//pk4hfmbb

#define FAN_LEVEL_ID  ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID //ESP_ZB_HA_THERMOSTAT_DEVICE_ID //ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID



#if(MAX_EP_SIZE == 1)
uint8_t ENDPOINTS_LIST[MAX_EP_SIZE] = {HA_ESP_LIGHT_ENDPOINT1};
uint32_t ENDPOINTS_TYPE[MAX_EP_SIZE] = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
#elif(MAX_EP_SIZE == 2)
uint8_t ENDPOINTS_LIST[MAX_EP_SIZE] = {HA_ESP_LIGHT_ENDPOINT1, HA_ESP_LIGHT_ENDPOINT2};
uint32_t ENDPOINTS_TYPE[MAX_EP_SIZE] = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, FAN_LEVEL_ID};
#elif(MAX_EP_SIZE == 3)
uint8_t ENDPOINTS_LIST[MAX_EP_SIZE] = {HA_ESP_LIGHT_ENDPOINT1, HA_ESP_LIGHT_ENDPOINT2, HA_ESP_LIGHT_ENDPOINT3};
uint32_t ENDPOINTS_TYPE[MAX_EP_SIZE] = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, FAN_LEVEL_ID};
#elif(MAX_EP_SIZE == 4)
uint8_t ENDPOINTS_LIST[MAX_EP_SIZE] = {HA_ESP_LIGHT_ENDPOINT1, HA_ESP_LIGHT_ENDPOINT2, HA_ESP_LIGHT_ENDPOINT3, HA_ESP_LIGHT_ENDPOINT4};
uint32_t ENDPOINTS_TYPE[MAX_EP_SIZE] = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, FAN_LEVEL_ID};
#elif(MAX_EP_SIZE == 5)
uint8_t ENDPOINTS_LIST[MAX_EP_SIZE] = {HA_ESP_LIGHT_ENDPOINT1, HA_ESP_LIGHT_ENDPOINT2, HA_ESP_LIGHT_ENDPOINT3, HA_ESP_LIGHT_ENDPOINT4, HA_ESP_LIGHT_ENDPOINT5};
uint32_t ENDPOINTS_TYPE[MAX_EP_SIZE] = {ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID};
#elif(MAX_EP_SIZE == 6)
uint8_t ENDPOINTS_LIST[MAX_EP_SIZE] = {HA_ESP_LIGHT_ENDPOINT1, HA_ESP_LIGHT_ENDPOINT2, HA_ESP_LIGHT_ENDPOINT3, HA_ESP_LIGHT_ENDPOINT4, HA_ESP_LIGHT_ENDPOINT5, HA_ESP_LIGHT_ENDPOINT6};
#else
uint8_t ENDPOINTS_LIST[MAX_EP_SIZE] = {HA_ESP_LIGHT_ENDPOINT1};
#endif

#define ESP_ZB_ZR_CONFIG()                                                              \
    {                                                                                   \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                               \
        .nwk_cfg.zczr_cfg = {                                                           \
            .max_children = MAX_CHILDREN,                                               \
        },                                                                              \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = RADIO_MODE_NATIVE,                        \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,      \
    }


//static const char *TAG = "ESP_TL_ON_OFF_LIGHT";

/* Define Button function currently only 1 switch define */
static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

static switch_func_pair_t button_func_pair1[] = {
 	{GPIO_INPUT_IO_TOGGLE_SWITCH1, SWITCH_ONOFF_TOGGLE_CONTROL}
};
static switch_func_pair_t button_func_pair2[] = {
 	{GPIO_INPUT_IO_TOGGLE_SWITCH2, SWITCH_ONOFF_TOGGLE_CONTROL}
};
static switch_func_pair_t button_func_pair3[] = {
 	{GPIO_INPUT_IO_TOGGLE_SWITCH3, SWITCH_ONOFF_TOGGLE_CONTROL}
};
static switch_func_pair_t button_func_pair4[] = {
 	{GPIO_INPUT_IO_TOGGLE_SWITCH4, SWITCH_ONOFF_TOGGLE_CONTROL}
};


//static bool light_state = false;
//static uint8_t switch1_state = 0;
//static uint8_t switch2_state = 0;
//static uint8_t switch3_state = 0;
//static uint8_t switch4_state = 0;
//static uint8_t light_state   = 0;

//bool light_state[4] = {false, false, false, false};
//uint8_t light_level[4] = {0, 0, 0, 0};
//uint16_t light_color_x[4] = {0, 0, 0, 0};
//uint16_t light_color_y[4] = {0, 0, 0, 0};

#define CONFIG_EXAMPLE_STRIP_LED_GPIO4   10
#define CONFIG_EXAMPLE_STRIP_LED_GPIO3   11
#define CONFIG_EXAMPLE_STRIP_LED_GPIO1   12
#define CONFIG_EXAMPLE_STRIP_LED_GPIO2   22

uint8_t gpio_led[4] = { CONFIG_EXAMPLE_STRIP_LED_GPIO1,CONFIG_EXAMPLE_STRIP_LED_GPIO2,
						CONFIG_EXAMPLE_STRIP_LED_GPIO3, CONFIG_EXAMPLE_STRIP_LED_GPIO4};

#define SHOW_MULTI_LIGHTS




