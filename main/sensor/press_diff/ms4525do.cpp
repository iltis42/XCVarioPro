#include "ms4525do.h"

#include "setup/SetupNG.h"
#include "logdef.h"

#include <I2Cbus.hpp>

#include <cmath>

#define MAX_AUTO_CORRECTED_OFFSET 50


constexpr char I2C_ADDRESS_MS4525DO   = 0x28;    /**< 7-bit address =0x28. 8-bit is 0x50. Depends on the order code (this is for code "I") */
 
// Register address
// #define ADDR_READ_MR            0x00    /* write to this address to start conversion */
 
// MS4525D sensor full scale range and units
// const int16_t MS4525FullScaleRange = 1;  // 1 psi
 
// MS4525D Sensor type (A or B) comment out the wrong type assignments
// Type A (10% to 90%)
const int16_t MS4525MinScaleCounts = 1638;
const int16_t MS4525FullScaleCounts = 14746;
  
const int16_t MS4525Span = MS4525FullScaleCounts - MS4525MinScaleCounts;
 
// MS4525D sensor differential pressure
// const int16_t MS4525ZeroCounts = (MS4525MinScaleCounts + MS4525FullScaleCounts) / 2;

MS4525DO::MS4525DO() : AsSensI2c(&i2c1, I2C_ADDRESS_MS4525DO)
{
    changeConfig();
}


void MS4525DO::changeConfig(){
	_multiplier = (2. * 6894.76 / MS4525Span) * ((100.0 + speedcal.get()) / 100.0);
}

float MS4525DO::getTemperature(void)
{
    float temp = t_dat;
    temp = temp / 10;          // now in deg F
    temp = (temp - 32) / 1.8f; // now in deg C
    return temp;
}


bool MS4525DO::offsetPlausible(uint32_t offset)
{
    ESP_LOGI(FNAME, "MS4525DO offsetPlausible(%ld)", offset);
    constexpr int lower_val = 7700;
    constexpr int upper_val = 8300;
    return (offset > lower_val) && (offset < upper_val);
}

int MS4525DO::getMaxACOffset()
{
    return MAX_AUTO_CORRECTED_OFFSET;
}
