/*C requirements*/
#include <stdio.h>

/* System includes*/
#include <pthread.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

/*utilitys*/
#include "esp_pthread.h"

/*internal drivers*/
#include "driver/gpio.h"
#include "driver/ledc.h"

/*external components*/
#include "TCS34725.h"

/*global*/
#include "globals.h"

static const char *TAG = "Info";
esp_err_t status;

#define LED_GPIO 2
#define SAMPLING_LED 27
#define CONFIG_BLINK_PERIOD 500

gpio_config_t led_sense =
{
    .pin_bit_mask = BIT64(5),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = 0,
    .pull_down_en = 0,
    .intr_type = GPIO_INTR_DISABLE
};

#ifdef _DRIVER_I2C_H_

#else
i2c_master_bus_config_t i2c_mst_cfg =
{
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 21,    
    .scl_io_num = 22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    //.intr_priority = 0,
    //.trans_queue_depth = 1,
    .flags.enable_internal_pullup = 1,    
};

i2c_device_config_t tcs1_cfg = 
{
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x29, //TCS34725_ADDRESS,
    .scl_speed_hz = 100000,
};

i2c_master_bus_handle_t bus0;
i2c_master_dev_handle_t tsc1_handle;
#endif

static uint8_t s_led_state = 0;

static void compile_time_set(void)
{
    time_t t_set;
    struct tm tmFw;
    char strftime_buf[32];
    
    /*Get time at compile*/
    strptime(__DATE__, "%b %d %Y", &tmFw);
    strptime(__TIME__, "%T", &tmFw);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &tmFw);
    t_set = mktime(&tmFw);
    struct timeval set_now = { .tv_sec = t_set };
    settimeofday(&set_now, NULL);
    ESP_LOGI(TAG, "Compile local date/time is: %s", strftime_buf);
}

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(LED_GPIO, s_led_state);
}

static void configure_led(void)
{
    //ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

static void sense_led(void)
{
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
    if(!gpio_get_level(5))
    {
         gpio_set_level(LED_GPIO, 1);
    } else {
        gpio_set_level(LED_GPIO, 0);
    }
}

static int64_t ms_schedule(void){
    struct timeval rtc_schedule;
    gettimeofday(&rtc_schedule, NULL);
    return (int64_t)rtc_schedule.tv_sec * 1000L + ((int64_t)rtc_schedule.tv_usec/1000);    
}

static void tune(void){

}

static void black_reference(uint16_t *r,uint16_t *g,uint16_t *b,uint16_t *c){

}

static void white_reference(uint16_t *r,uint16_t *g,uint16_t *b,uint16_t *c){

}

void app_main(void)
{
    compile_time_set();    

    /* Configure the peripheral according to the LED type */
    configure_led();
    gpio_reset_pin(SAMPLING_LED);
    gpio_set_direction(SAMPLING_LED, GPIO_MODE_OUTPUT);

    gpio_reset_pin(5);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_config(&led_sense);

    gpio_reset_pin(i2c_mst_cfg.scl_io_num);
    gpio_reset_pin(i2c_mst_cfg.sda_io_num);
    gpio_set_pull_mode(i2c_mst_cfg.scl_io_num,GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(i2c_mst_cfg.sda_io_num,GPIO_PULLUP_ONLY);
    gpio_set_direction(i2c_mst_cfg.scl_io_num,GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(i2c_mst_cfg.sda_io_num,GPIO_MODE_INPUT_OUTPUT);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(status = i2c_new_master_bus(&i2c_mst_cfg,&bus0));
    if(status == ESP_OK) ESP_LOGI(TAG, "I2C bus initialized successfully");
    
    ESP_ERROR_CHECK(status = i2c_master_probe(bus0, tcs1_cfg.device_address, -1));
    if(status == ESP_OK)
    {
        ESP_LOGI(TAG, "Found TSC module");  
    } else {
        ESP_LOGI(TAG, "NO TSC module");
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(status = i2c_master_bus_add_device(bus0, &tcs1_cfg, &tsc1_handle));
    if(status == ESP_OK)
    {
        ESP_LOGI(TAG, "TSC 1 initialized successfully");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    TSC34725_init();
        
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /*ESP_ERROR_CHECK(status = i2c_del_master_bus(bus0));
    if(status == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C bus closed");  
    } else {
        ESP_LOGI(TAG, "I2C bus closure failure");
    }*/

    /* white back ground 226,222,219  #E2DEDB*/
    uint16_t r=0,g=0,b=0,c=0;

    TSC34725_setGain(TCS34725_GAIN_16X);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    TSC34725_setIntegrationTime(0x35);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    TSC34725_getRawData(&r,&g,&b,&c);
    ESP_LOGI(TAG, "Red: %d, Green: %d, Blue: %d, Clear: %d", r,g,b,c);
    gpio_set_level(SAMPLING_LED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    TSC34725_getRawData(&r,&g,&b,&c);
    ESP_LOGI(TAG, "Red: %d, Green: %d, Blue: %d, Clear: %d", r,g,b,c);

    uint16_t cctemp = TSC34725_calculateColorTemperature_dn40(r,g,b,c);
    ESP_LOGI(TAG, "color temp: %d", cctemp);
    float rf=0,gf=0,bf=0;
    getRGB(&rf,&gf,&bf);
    ESP_LOGI(TAG, "Red: %.2f, Green: %.2f, Blue: %.2f, Clear: %d", rf,gf,bf,c);

    static int64_t time_ms = 0, lastTime = 0;
    uint8_t bubbles = 0;
    while (1) {
        time_ms = ms_schedule();
        vTaskDelay(1 / portTICK_PERIOD_MS);

        if(time_ms > (lastTime + CONFIG_BLINK_PERIOD)){
            s_led_state = !s_led_state;
            blink_led();
            lastTime = time_ms;
        }

        /*if(!gpio_get_level(5))
        {
            bubbles++;
            gpio_set_level(BLINK_GPIO, 1);
        } else {
            gpio_set_level(BLINK_GPIO, 0);
        }
            
        if(time_ms >= (lastTime + 5000))
        { 
            
            ESP_LOGI(TAG, "Bubbles per second: %.2f", (double)(bubbles / 5));
            bubbles = 0;
            lastTime = time_ms;
        }*/
    }
}


/**/