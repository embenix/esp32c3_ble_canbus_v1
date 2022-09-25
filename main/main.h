/**
 * @file main.h
 * @author Yasir K. Qureshi
 *
 */

#ifndef MAIN_H
#define MAIN_H

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>


/*********************
 *      DEFINES
 *********************/
static const char * PROJECT_TAG = "BLE-CAN_Unit";
#define PERI_PWR_EN_PIN         7  //Set this Pin Low to enable the power to the on board peripherals
#define CAN_STB_PIN             4  //HIGH =  CAN Sleep | LOW = CAN Active
#define GPIO_OUTPUT_PIN_SEL     ((1ULL<<PERI_PWR_EN_PIN) | (1ULL<<CAN_STB_PIN))
#define BUTTON_1_PIN            9  //Active Low | Input pin
#define BUTTON_2_PIN            10 //Active Low | Input pin
#define GPIO_INPUT_PIN_SEL      ((1ULL<<BUTTON_1_PIN) | (1ULL<<BUTTON_2_PIN))
#define ESP_INTR_FLAG_DEFAULT   0
#define I2C_SDA_PIN             CONFIG_I2C_SDA_GPIO_NUM
#define I2C_SCL_PIN             CONFIG_I2C_SCL_GPIO_NUM
#define WS2812_RGB_PIN          CONFIG_LEDS_RMT_TX_GPIO

#define RMT_TX_CHANNEL RMT_CHANNEL_0

/**********************
 *      TYPEDEFS
 **********************/
struct AMessage {
    uint8_t idx;
    uint8_t color;
    uint32_t red;
    uint32_t green;
    uint32_t blue;
};

/**********************
 * GLOBAL PROTOTYPES
 **********************/
static void init_gpio(void);
static void init_i2c(void);
static void ws2812_led_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);
static void led_control_task(void* ignore);
static void enable_peripheral_power(void);
static void disable_peripheral_power(void);

static void gpio_task_example(void* ignore);
static void IRAM_ATTR gpio_isr_handler(void* arg);


/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*CANBUS_TASK_H*/