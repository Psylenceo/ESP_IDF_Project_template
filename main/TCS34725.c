/*C requirements*/
#include <stdio.h>
#include <math.h>

/*system*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*External devices*/
#include "TCS34725.h"

/*global values*/
#include "globals.h"

/*constants*/

/*I2C device*/
uint8_t _addr;
//TwoWire *_wire;
bool _begun;

/*adafruit TCS*/
//I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
bool _tcs34725Initialised;
tcs34725Gain_t _tcs34725Gain;
uint8_t _tcs34725IntegrationTime;

static const char *TAG = "TCS34725 info:";

float powf(const float x, const float y) {
  return (float)(pow((double)x, (double)y));
}

void TSC34725_write8(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg),value};
    ESP_ERROR_CHECK(i2c_master_transmit(tsc1_handle,buffer,sizeof(buffer),100));
}

uint8_t TSC34725_read8(uint8_t reg)
{
    uint8_t buf[1] = {(uint8_t)(TCS34725_COMMAND_BIT | reg)};
    uint8_t buffer[2] = {};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tsc1_handle,buf,sizeof(buf),buffer,2,100));
    return buffer[0];
}

uint16_t TSC34725_read16(uint8_t reg)
{
    uint8_t buf[1] = {(uint8_t)(TCS34725_COMMAND_BIT | reg)};
    uint8_t buffer[2] = {};
    i2c_master_transmit_receive(tsc1_handle,buf,sizeof(buf),buffer,sizeof(buffer),100);
    return ((uint16_t)buffer[1] << 8) | ((uint16_t)buffer[0] & 0xFF);
}

void TSC34725_enable(void)
{
    TSC34725_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    vTaskDelay(30 / portTICK_PERIOD_MS);
    TSC34725_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
    /* 12/5 = 2.4, add 1 to account for integer truncation */
    vTaskDelay(((256 - _tcs34725IntegrationTime) * 12 / 5 + 1)/portTICK_PERIOD_MS);
}

void TSC34725_disable(void)
{
    /* Turn the device off to save power */
    uint8_t reg = 0;
    reg = TSC34725_read8(TCS34725_ENABLE);
    TSC34725_write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

bool TSC34725_init()
{
    //if(!bus_link->begin) return false

    /* Make sure we're actually connected */
    uint8_t x = TSC34725_read8(TCS34725_ID);
    ESP_LOGI(TAG, "ID: 0x%x",x);
    if ((x != 0x4d) && (x != 0x44) && (x != 0x10)) {
        return false;
    }
    _tcs34725Initialised = true;
    ESP_LOGI(TAG, "Initialized sest");

    /* Set default integration time and gain */
    TSC34725_setIntegrationTime(_tcs34725IntegrationTime);
    TSC34725_setGain(_tcs34725Gain);

    /* Note: by default, the device is in power down mode on bootup */
    TSC34725_enable();

    return true;
}

bool TSC34725_begin(uint8_t addr, i2c_master_bus_handle_t *bus) //com_bus won't really be usedd atm, placeholder
{
    //if(bus_link) delete bus_link
    //i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    return TSC34725_init(); 
}

void TSC34725_setIntegrationTime(uint8_t it) {
    //if (!_tcs34725Initialised) begin();

    /* Update the timing register */
    TSC34725_write8(TCS34725_ATIME, it);

    /* Update value placeholders */
    _tcs34725IntegrationTime = it;
}

void TSC34725_setGain(uint8_t gain) {
    //if (!_tcs34725Initialised) begin();

    /* Update the timing register */
    TSC34725_write8(TCS34725_CONTROL, gain);

    /* Update value placeholders */
    _tcs34725Gain = gain;
}

void TSC34725_getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) 
{
    //if (!_tcs34725Initialised) begin();

    *c = TSC34725_read16(TCS34725_CDATAL);
    *r = TSC34725_read16(TCS34725_RDATAL);
    *g = TSC34725_read16(TCS34725_GDATAL);
    *b = TSC34725_read16(TCS34725_BDATAL);

    /* Set a delay for the integration time */
    /* 12/5 = 2.4, add 1 to account for integer truncation */
    vTaskDelay(((256 - _tcs34725IntegrationTime) * 12 / 5 + 1) / portTICK_PERIOD_MS);
}

void TSC34725_getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    //if (!_tcs34725Initialised) begin();

    TSC34725_enable();
    TSC34725_getRawData(r, g, b, c);
    TSC34725_disable();
}

void getRGB(float *r, float *g, float *b) 
{
    uint16_t red, green, blue, clear;
    TSC34725_getRawData(&red, &green, &blue, &clear);
    uint32_t sum = clear;

    // Avoid divide by zero errors ... if clear = 0 return black
    if (clear == 0) {
        *r = *g = *b = 0;
        return;
    }

    *r = (float)red / sum * 255.0;
    *g = (float)green / sum * 255.0;
    *b = (float)blue / sum * 255.0;
}

uint16_t TSC34725_calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
    float X, Y, Z; /* RGB to XYZ correlation      */
    float xc, yc;  /* Chromaticity co-ordinates   */
    float n;       /* McCamy's formula            */
    float cct;

    if (r == 0 && g == 0 && b == 0) {
        return 0;
    }

    /* 1. Map RGB values to their XYZ counterparts.    */
    /* Based on 6500K fluorescent, 3000K fluorescent   */
    /* and 60W incandescent values for a wide range.   */
    /* Note: Y = Illuminance or lux                    */
    X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
    Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
    Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

    /* 2. Calculate the chromaticity co-ordinates      */
    xc = (X) / (X + Y + Z);
    yc = (Y) / (X + Y + Z);

    /* 3. Use McCamy's formula to determine the CCT    */
    n = (xc - 0.3320F) / (0.1858F - yc);

    /* Calculate the final CCT */
    cct =
        (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

    /* Return the results in degrees Kelvin */
    return (uint16_t)cct;
}

uint16_t TSC34725_calculateColorTemperature_dn40(uint16_t r, uint16_t g, uint16_t b, uint16_t c) 
{
    uint16_t r2, b2; /* RGB values minus IR component */
    uint16_t sat;    /* Digital saturation level */
    uint16_t ir;     /* Inferred IR content */

    if (c == 0) {
        return 0;
    }

    /* Analog/Digital saturation:
    *
    * (a) As light becomes brighter, the clear channel will tend to
    *     saturate first since R+G+B is approximately equal to C.
    * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
    *     time, up to a maximum values of 65535. This means analog
    *     saturation can occur up to an integration time of 153.6ms
    *     (64*2.4ms=153.6ms).
    * (c) If the integration time is > 153.6ms, digital saturation will
    *     occur before analog saturation. Digital saturation occurs when
    *     the count reaches 65535.
    */
    if ((256 - _tcs34725IntegrationTime) > 63) {
        /* Track digital saturation */
        sat = 65535;
    } else {
        /* Track analog saturation */
        sat = 1024 * (256 - _tcs34725IntegrationTime);
    }

    /* Ripple rejection:
    *
    * (a) An integration time of 50ms or multiples of 50ms are required to
    *     reject both 50Hz and 60Hz ripple.
    * (b) If an integration time faster than 50ms is required, you may need
    *     to average a number of samples over a 50ms period to reject ripple
    *     from fluorescent and incandescent light sources.
    *
    * Ripple saturation notes:
    *
    * (a) If there is ripple in the received signal, the value read from C
    *     will be less than the max, but still have some effects of being
    *     saturated. This means that you can be below the 'sat' value, but
    *     still be saturating. At integration times >150ms this can be
    *     ignored, but <= 150ms you should calculate the 75% saturation
    *     level to avoid this problem.
    */
    if ((256 - _tcs34725IntegrationTime) <= 63) {
        /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
        sat -= sat / 4;
    }

    /* Check for saturation and mark the sample as invalid if true */
    if (c >= sat) {
        return 0;
    }

    /* AMS RGB sensors have no IR channel, so the IR content must be */
    /* calculated indirectly. */
    ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

    /* Remove the IR component from the raw RGB values */
    r2 = r - ir;
    b2 = b - ir;

    if (r2 == 0) {
        return 0;
    }

    /* A simple method of measuring color temp is to use the ratio of blue */
    /* to red light, taking IR cancellation into account. */
    uint16_t cct = (3810 * (uint32_t)b2) / /** Color temp coefficient. */
                        (uint32_t)r2 +
                    1391; /** Color temp offset. */

    return cct;
}

uint16_t TSC34725_calculateLux(uint16_t r, uint16_t g, uint16_t b) 
{
    float illuminance;

    /* This only uses RGB ... how can we integrate clear or calculate lux */
    /* based exclusively on clear since this might be more reliable?      */
    illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

    return (uint16_t)illuminance;
}

void TSC34725_setInterrupt(bool i) 
{
    uint8_t r = TSC34725_read8(TCS34725_ENABLE);
    if (i) {
        r |= TCS34725_ENABLE_AIEN;
    } else {
        r &= ~TCS34725_ENABLE_AIEN;
    }
    TSC34725_write8(TCS34725_ENABLE, r);
}

void TSC34725_clearInterrupt() 
{
    uint8_t buffer[1] = {TCS34725_COMMAND_BIT | 0x66};
    ESP_ERROR_CHECK(i2c_master_transmit(tsc1_handle,buffer,sizeof(buffer),100));
}

void TSC34725_setIntLimits(uint16_t low, uint16_t high) {
    TSC34725_write8(0x04, low & 0xFF);
    TSC34725_write8(0x05, low >> 8);
    TSC34725_write8(0x06, high & 0xFF);
    TSC34725_write8(0x07, high >> 8);
}

