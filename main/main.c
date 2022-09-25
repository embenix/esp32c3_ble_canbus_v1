#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"
#include "driver/twai.h"
#include "canbus_task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "main.h"
#include "sht3x.h"

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle ledQueue = NULL;
static sht3x_sensor_t* sensor;    // sensor device data structure

#define ESP_PLATFORM

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

#else  // ESP8266 (esp-open-rtos)

// user task stack depth for ESP8266
#define TASK_STACK_DEPTH 256

#endif  // ESP_PLATFORM

void sensiron_task (void *pvParameters)
{
    float temperature;
    float humidity;

    // Start periodic measurements with 1 measurement per second.
    sht3x_start_measurement (sensor, sht3x_periodic_1mps, sht3x_high);

    // Wait until first measurement is ready (constant time of at least 30 ms
    // or the duration returned from *sht3x_get_measurement_duration*).
    vTaskDelay (sht3x_get_measurement_duration(sht3x_high));

    TickType_t last_wakeup = xTaskGetTickCount();
    
    while (1) 
    {
        // Get the values and do something with them.
        if (sht3x_get_results (sensor, &temperature, &humidity))
            printf("%.3f SHT3x Sensor: %.2f °C, %.2f %%\n", 
                   (double)sdk_system_get_time()*1e-3, temperature, humidity);
                   
        // Wait until 2 seconds (cycle time) are over.
        vTaskDelayUntil(&last_wakeup, 2000 / portTICK_PERIOD_MS);
    }
}

/*
    MAIN Method
*/
void app_main(void)
{
    //Init GPIO
    init_gpio();

    //Init I2C
    init_i2c();

    gpio_set_level(PERI_PWR_EN_PIN, 0);
    gpio_set_level(CAN_STB_PIN, 0);

    //Create the sensors, multiple sensors are possible.
    if ((sensor = sht3x_init_sensor (I2C_NUM_0, SHT3x_ADDR_1)))
    {
        // Create a user task that uses the sensors.
        xTaskCreatePinnedToCore(sensiron_task, "sensiron_task", TASK_STACK_DEPTH, NULL, 2, NULL, 1);
    }
    else {
        //printf("Could not initialize SHT3x sensor\n");
        ESP_LOGE("Sensiron_SHT30", "Install SHT3x driver failed!");
    }

    //Initialize the tasks
    xTaskCreatePinnedToCore(led_control_task, "LED_control_task", 4096, NULL, 2, NULL, 1); //4096 //led_control_task();
    can_task();

    //
    ledQueue = xQueueCreate(30, sizeof( struct AMessage *));
    struct AMessage ledMsg = {
        .idx = 0,
        .color = 0,
        .red = 0,
        .green = 0,
        .blue = 0
    };

    uint32_t hue = 0;
    while(true){
        //ws2812_led_hsv2rgb(hue++, 100,  20, &ledMsg.red, &ledMsg.green, &ledMsg.blue);
        //if (hue >= 360) hue = 0;

        //ledMsg.red = 10 - ledMsg.red;
        //ledMsg.green = 10 - ledMsg.green;
        //xQueueSend(ledQueue, &ledMsg, (TickType_t) 0);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
static void ws2812_led_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

/**
 * @brief Control the Serial RGB LED
 * 
 * @param ignore 
 * @return nothing 
 */
static void led_control_task(void* ignore)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(WS2812_RGB_PIN, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_LEDS_COUNT, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE("WS2812_LED", "Install WS2812 driver failed!");
    }
    else {
        ESP_LOGI("WS2812_LED", "Install WS2812 driver successful.");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    // Show simple rainbow chasing pattern
    //ESP_LOGI(TAG, "LED Rainbow Chase Start");

    struct AMessage ledMsg = {
        .idx = 0,
        .color = 0,
        .red = 0,
        .green = 0,
        .blue = 0
    };
    

    while (true) {
        // for (int i = 0; i < 3; i++) {
        //     for (int j = i; j < CONFIG_LEDS_COUNT; j += 3) {
        //         // Build RGB values
        //         hue = j * 360 / CONFIG_LEDS_COUNT + start_rgb;
        //         led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
        //         // Write RGB values to strip driver
        //         ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
        //     }
        //     // Flush RGB values to LEDs
        //     ESP_ERROR_CHECK(strip->refresh(strip, 100));
        //     vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        //     strip->clear(strip, 50);
        //     vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        // }
        // start_rgb += 60;

        // if (ledQueue != 0){
        //     if (xQueueReceive(ledQueue, &ledMsg, (TickType_t) 5)) {

        //         ESP_LOGI(PROJECT_TAG, "LED data received: Index=%u | Color=%u  | R=%lu | G=%lu | B=%lu", ledMsg.idx, ledMsg.color, ledMsg.red, ledMsg.green, ledMsg.blue);
        //         //ESP_ERROR_CHECK(strip->clear(strip, 50));
        //         //vTaskDelay(pdMS_TO_TICKS(10));
        //         ESP_ERROR_CHECK(strip->set_pixel(strip, ledMsg.idx, ledMsg.red, ledMsg.green, ledMsg.blue));
        //         //ESP_ERROR_CHECK(strip->set_pixel(strip, ledMsg.idx, colorRed[ledMsg.color], colorGreen[ledMsg.color], colorBlue[ledMsg.color]));
        //         ESP_ERROR_CHECK(strip->refresh(strip, 100));
        //         vTaskDelay(pdMS_TO_TICKS(10));
        //     }
        // }
        if (++hue >= 240) hue = 0;
        ws2812_led_hsv2rgb(hue, 100,  2, &ledMsg.red, &ledMsg.green, &ledMsg.blue);
        ESP_ERROR_CHECK(strip->set_pixel(strip, ledMsg.idx, ledMsg.red, ledMsg.green, ledMsg.blue));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

/*
    Initialize the GPIO Pins | Buttons | Control Pins
*/
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_evt_queue != 0){
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
}
static void gpio_task_example(void* ignore)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == 9) printf("SW1 - GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            else if (io_num == 10) printf("SW2 - GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}
static void init_gpio(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that we set as ouptut e.g. peripheral Power power
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

     //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;//  GPIO_INTR_POSEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_36, GPIO_INTR_ANYEDGE);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_1_PIN, gpio_isr_handler, (void*) BUTTON_1_PIN);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_2_PIN, gpio_isr_handler, (void*) BUTTON_2_PIN);
    // //hook isr handler for specific gpio pin
    // gpio_isr_handler_add(GPIO_INPUT_IO_38, gpio_isr_handler, (void*) GPIO_INPUT_IO_38);
    // //hook isr handler for specific gpio pin
    // gpio_isr_handler_add(GPIO_INPUT_IO_39, gpio_isr_handler, (void*) GPIO_INPUT_IO_39);

    ESP_LOGI(PROJECT_TAG,"Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
}


/**
 * @brief Initialize I2C driver.
 */
static void init_i2c(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };

    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}


/**
 * @brief Read the humidiy and temperature from the SHT30 sensor.
 */
static esp_err_t read_sht30_Sensor(i2c_port_t i2c_num, uint8_t *data)
{
    // int ret;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if (ret != ESP_OK) {
    //     return ret;
    // }
    // vTaskDelay(30 / portTICK_RATE_MS);
    // cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    // i2c_master_read_byte(cmd, data_h, ACK_VAL);
    // i2c_master_read_byte(cmd, data_l, NACK_VAL);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // return ret;

    // int ret;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, 0x44, true);
    // i2c_master_write_byte(cmd, 0x2C, true);
    // i2c_master_write_byte(cmd, 0x06, true);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    // vTaskDelay(30 / portTICK_RATE_MS);
    // cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, data[0], true);
    // i2c_master_read_byte(cmd, data_h, ACK_VAL);
    // i2c_master_read_byte(cmd, data_l, NACK_VAL);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    // return ret;
    return 0;
}