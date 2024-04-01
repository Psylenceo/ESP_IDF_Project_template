/*C requirements*/
#include <stdio.h>
#include <math.h>

/*system*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*self*/
#include "I2C_wrapper.h"

/*External devices*/


/*global values*/






    #ifdef _DRIVER_I2C_H_

    #else
    /*ESP_ERROR_CHECK(status = i2c_new_master_bus(&i2c_mst_cfg,&bus0));
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    ESP_ERROR_CHECK(i2c_master_bus_reset(bus0));
    ESP_ERROR_CHECK(status = i2c_master_bus_add_device(bus0, &tcs1_cfg, &tsc1_handle));
    ESP_LOGI(TAG, "TSC 1 initialized successfully");

    ESP_ERROR_CHECK(status = i2c_master_probe(bus0, tcs1_cfg.device_address, 30));
    if(status == ESP_OK)
    {
        ESP_LOGI(TAG, "Found TSC module");  
    } else {
        ESP_LOGI(TAG, "NO TSC module");
    }

    //ESP_ERROR_CHECK(status = TSC34725_init());
    if(status)
    {
        ESP_LOGI(TAG, "My file worked");  
    } else {
        ESP_LOGI(TAG, " Did not work");
    }
    
    ESP_ERROR_CHECK(status = i2c_del_master_bus(bus0));
    if(status == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C bus closed");  
    } else {
        ESP_LOGI(TAG, "I2C bus closure failure");
    }*/
    #endif

