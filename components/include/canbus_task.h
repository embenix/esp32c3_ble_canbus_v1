/**
 * @file canbus_task.h
 * @author Yasir K. Qureshi
 *
 */

#ifndef CANBUS_TASK_H
#define CANBUS_TASK_H

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"


// QueueSetHandle_t ledQueue;

// struct AMessage {
//     uint8_t idx;
//     uint8_t color;
//     uint8_t red;
//     uint8_t green;
//     uint8_t blue;
// };

/*********************
 *      DEFINES
 *********************/
#define NO_OF_MSGS                      10
#define NO_OF_ITERS                     3
#define RX_TASK_PRIO                    9
#define TX_GPIO_NUM                     CONFIG_CAN_TX_GPIO_NUM
#define RX_GPIO_NUM                     CONFIG_CAN_RX_GPIO_NUM
#define TX_TASK_PRIO                    8       //Sending task priority
#define RX_TASK_PRIO                    9       //Receiving task priority
#define CTRL_TSK_PRIO                   10      //Control task priority
#define MSG_ID                          21     //11 bit standard format ID
static const char * CAN_TAG = "TWAI";

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

/**********************
 *      TYPEDEFS
 **********************/
static SemaphoreHandle_t tx_sem;
static SemaphoreHandle_t rx_sem;
SemaphoreHandle_t ctrl_sem;
static SemaphoreHandle_t done_sem;
QueueSetHandle_t canTxQueue;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void can_task(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*CANBUS_TASK_H*/