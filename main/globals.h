#pragma once

/*System includes*/
#include "esp_log.h"

/*internal drivers*/
//#include "driver/i2c.h"
#include "driver/i2c_master.h"

/*debug enable*/
#define _DEBUG true
#define _LOGGING true

#ifdef _DRIVER_I2C_H_
extern i2c_port_t i2c_port;
extern i2c_config_t config_TCS_1;
extern i2c_cmd_handle_t i2c_cmd;
#else
extern i2c_master_bus_config_t i2c_mst_cfg;
extern i2c_device_config_t tcs1_cfg;
extern esp_err_t status;
extern i2c_master_bus_handle_t bus0;
extern i2c_master_dev_handle_t tsc1_handle;
#endif

