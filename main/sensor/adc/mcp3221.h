#pragma once

#include "esp_err.h"

namespace i2cbus {
    class I2C;
}


// Connect module using I2C port pins sda and scl. The output is referenced to the supply voltage which can be
// 2.7v to 5.0v. The read will return the correct voltage, if you supply the correct supplyVoltage when instantiating.

class MCP3221
{
public:
    MCP3221(i2cbus::I2C *b);
    ~MCP3221() = default;

    // check for reply with I2C bus address
    esp_err_t selfTest();

    // raw read value of airspeed sensor
    int readVal();

    // Reads the analog register of the MCP3221 and converts it to a useable value. (a voltage)
    esp_err_t readRaw(uint16_t &val);

    // alpha = 1.0  means no filter
    //         0.1  means 10 samples until full value
    // float readAVG(float alpha);

private:
    i2cbus::I2C *_bus;
    const uint8_t _address;
    // float exponential_average;
};

