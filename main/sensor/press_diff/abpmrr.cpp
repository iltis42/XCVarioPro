#include "abpmrr.h"
#include "setup/SetupNG.h"
#include "logdef.h"

#include <I2Cbus.hpp>

#include <cmath>

#define I2C_ADDRESS_ABPMRR    0x28    ///< 7-bit address =0x28. 8-bit is 0x50. Depends on the order code (this is for code "I")


/* Register address */
// #define ADDR_READ_MR            0x00    /* write to this address to start conversion */
 
// ABPMRRD sensor full scale range and units
// const int16_t ABPMRRFullScaleRange = 1;  // 1 psi
 
// ABPMRRD Sensor type (10% to 90%)
// Output (% of 2^14 counts) = P max. 80% x (Pressure applied – P min. ) + 10%

const int16_t ABPMRRMinScaleCounts = 1638;
const int16_t ABPMRRFullScaleCounts = 14746;

const int16_t ABPMRRSpan = ABPMRRFullScaleCounts - ABPMRRMinScaleCounts;
 
// ABPMRRD sensor differential pressure style 
// const int16_t ABPMRRZeroCounts = (ABPMRRMinScaleCounts + ABPMRRFullScaleCounts) / 2;
 
#define MAX_AUTO_CORRECTED_OFFSET 50

ABPMRR::ABPMRR() : AsSensI2c(&i2c1, I2C_ADDRESS_ABPMRR)
{
    changeConfig();
}

void ABPMRR::changeConfig()
{
    _multiplier = (2.f * 6894.76 / ABPMRRSpan) * ((100.0 + speedcal.get()) / 100.0);
    ESP_LOGI(FNAME, "changeConfig, speed multiplier %f, speed cal: %f ", _multiplier, speedcal.get());
}

float ABPMRR::getTemperature()
{
	float temp = t_dat;
	temp = temp / 10;         // now in deg F
	temp = (temp -32) / 1.8f; // now in deg C
	return temp;
}

bool ABPMRR::offsetPlausible(uint32_t offset)
{
    ESP_LOGI(FNAME, "ABPMRR offsetPlausible( %ld )", offset);
    constexpr int lower_val = 8192 - 200;
    constexpr int upper_val = 8192 + 200;
    return (offset > lower_val) && (offset < upper_val);
}

int ABPMRR::getMaxACOffset()
{
    return MAX_AUTO_CORRECTED_OFFSET;
}
