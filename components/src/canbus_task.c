/**
 * @file canbus_task.c
 * @author Yasir K. Qureshi
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "canbus_task.h"
#include "driver/gpio.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void twai_transmit_task(void *arg);
static void twai_receive_task(void *arg);
static void twai_control_task(void *arg);

/**********************
 *  STATIC VARIABLES
 **********************/
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
//Set to NO_ACK mode due to self testing with single module
static const twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                               .tx_io = TX_GPIO_NUM,
                                               .rx_io = RX_GPIO_NUM,
                                               .clkout_io = TWAI_IO_UNUSED,
                                               .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 0,
                                               .rx_queue_len = 8,
                                               .alerts_enabled = TWAI_ALERT_NONE,
                                               .clkout_divider = 0};


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/* --------------------------- Tasks and Functions -------------------------- */
static void twai_transmit_task(void *arg)
{
    xSemaphoreTake(tx_sem, portMAX_DELAY);
    twai_message_t tx_msg = {.data_length_code = 8, .identifier = MSG_ID, .self = 0};
    uint8_t bID = 0;
    //for (int iter = 0; iter < NO_OF_ITERS; iter++) {
    while(1){
        //if (gpio_evt_queue != 0){
            //if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)){
                //ESP_LOGI(CAN_TAG,"GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
                // ESP_LOGI(CAN_TAG, "Button Touched: %c | %u | 0x%02x", bID, bID, bID);
                // bID = 0;

                // for (uint8_t x = 0; x < 4; x++){
                //     if (gpio_get_level(x+36) == 1) bID |= (1 << x);
                //     else bID &= ~(1 << x);
                // }

                // tx_msg.data[0] = '$';
                // //if (bID > 0) tx_msg.data[1] = (1 << (bID - 0x31));
                // //else tx_msg.data[1] = 0; //(1 << (bID - 0x31));
                // tx_msg.data[1] = bID;
                // tx_msg.data[2] = 0;
                // tx_msg.data[3] = 0;
                // tx_msg.data[4] = 1;
                // tx_msg.data[5] = 0xAA;
                // tx_msg.data[6] = (bID & 0xFF);
                // tx_msg.data[7] = 0;
                // ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
            //}
        //}
        vTaskDelay(pdMS_TO_TICKS(500));
        // xSemaphoreTake(tx_sem, portMAX_DELAY);
        // for (int i = 0; i < NO_OF_MSGS; i++) {
        //     //Transmit messages using self reception request
        //     tx_msg.data[0] = i;
        //     ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
        //     vTaskDelay(pdMS_TO_TICKS(10));
        // }
    }
    vTaskDelete(NULL);
}

static void twai_receive_task(void *arg)
{
    xSemaphoreTake(rx_sem, portMAX_DELAY);
    bool start_cmd = false;
    bool stop_resp = false;
    uint32_t iterations = 0;
    ESP_LOGI(CAN_TAG, "Starting to receive data...");

    // uint8_t ledColorBytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // struct AMessage ledMsg = {
    //     .idx = 0,
    //     .color = 0,
    //     .red = 0,
    //     .green = 0,
    //     .blue = 0
    // };

    while (iterations < NO_OF_ITERS)
    {
        twai_message_t rx_msg;
        ESP_ERROR_CHECK(twai_receive(&rx_msg, portMAX_DELAY));
        if (rx_msg.identifier == 21){
            uint32_t data = 0;
            char dataBuffer[50];
            data = sprintf(dataBuffer, "%02X %02X %02X %02X %02X %02X %02X %02X",
                        rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3], rx_msg.data[4],
                        rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);

            dataBuffer[data] = 0;
            ESP_LOGI(CAN_TAG, "Received data from: 0x%03X | %s", rx_msg.identifier, dataBuffer);


            // if (rx_msg.data[0] == 1){
            //     uint8_t tmp = 0;
            //     uint8_t tmpColorBytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

            //     for (int i = 1; i < 5; i++){
            //         tmpColorBytes[tmp] = ((rx_msg.data[i] & 0xF0) >> 4);
            //         tmpColorBytes[tmp+1] = (rx_msg.data[i] & 0x0F);

            //         if ((tmpColorBytes[tmp] > 0) && (tmpColorBytes[tmp] != ledColorBytes[tmp])){
            //             ledColorBytes[tmp] = tmpColorBytes[tmp];
            //             ledMsg.idx = tmp;
            //             ledMsg.color = tmpColorBytes[tmp];
            //             xQueueSend(ledQueue, &ledMsg, (TickType_t) 0);
            //         }

            //         if ((tmpColorBytes[tmp + 1] > 0) && (tmpColorBytes[tmp + 1] != ledColorBytes[tmp + 1])){
            //             ledColorBytes[tmp + 1] = tmpColorBytes[tmp + 1];
            //             ledMsg.idx = tmp + 1;
            //             ledMsg.color = tmpColorBytes[tmp + 1];
            //             xQueueSend(ledQueue, &ledMsg, (TickType_t) 0);
            //         }

            //         // if (((rx_msg.data[i] & 0xF0) >> 4) > 0) {
            //         //     ledMsg.idx = tmp;
            //         //     ledMsg.color = ((rx_msg.data[i] & 0xF0) >> 4);
            //         //     xQueueSend(ledQueue, &ledMsg, (TickType_t) 0);
            //         // }

            //         // if (((rx_msg.data[i] & 0x0F)) > 0) {
            //         //     ledMsg.idx = tmp+1;
            //         //     ledMsg.color = ((rx_msg.data[i] & 0x0F));
            //         //     xQueueSend(ledQueue, &ledMsg, (TickType_t) 0);
            //         // }

            //         tmp = tmp + 2;
            //     }
            // }

            twai_clear_receive_queue();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    xSemaphoreGive(rx_sem);
    vTaskDelete(NULL);
}

// static void twai_control_task(void *arg)
// {
//     while (1){
//     xSemaphoreTake(ctrl_sem, portMAX_DELAY);
//     //for (int iter = 0; iter < NO_OF_ITERS; iter++) {
//         //Start TWAI Driver for this iteration
//         ESP_ERROR_CHECK(twai_start());
//         ESP_LOGI(CAN_TAG, "TWAI Driver started");

//         //Trigger TX and RX tasks to start transmitting/receiving
//         //xSemaphoreGive(rx_sem);
//         //xSemaphoreGive(tx_sem);
//         xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for TX and RX tasks to finish iteration

//         ESP_ERROR_CHECK(twai_stop());               //Stop the TWAI Driver
//         ESP_LOGI(CAN_TAG, "TWAI Driver stopped");
//         vTaskDelay(pdMS_TO_TICKS(100));             //Delay then start next iteration
//     }
//     xSemaphoreGive(done_sem);
//     vTaskDelete(NULL);
// }

void can_task(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));             //Delay then start next iteration

    rx_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 8192, NULL, RX_TASK_PRIO, NULL, 1); //4096
    tx_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 8192, NULL, TX_TASK_PRIO, NULL, 1); //4096

    //Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_TAG, "Driver started");

    xSemaphoreGive(rx_sem);                     //Start RX task
    xSemaphoreGive(tx_sem);                     //Start TX task
    vTaskDelay(pdMS_TO_TICKS(100));
    // xSemaphoreTake(rx_sem, portMAX_DELAY);      //Wait for RX task to complete

    // //Stop and uninstall TWAI driver
    // ESP_ERROR_CHECK(twai_stop());
    // ESP_LOGI(CAN_TAG, "Driver stopped");
    // ESP_ERROR_CHECK(twai_driver_uninstall());
    // ESP_LOGI(CAN_TAG, "Driver uninstalled");

    // //Cleanup
    // vSemaphoreDelete(rx_sem);
}