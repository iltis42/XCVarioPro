
#pragma once

#include "AsSensI2c.h"

#include "I2Cbus.hpp"

#include <cstdint>

class MCPH21 final : public AsSensI2c
{
public:
    // instance methods
    MCPH21();
    ~MCPH21() = default;

    const char *name() const override { return "MCPH21"; }
    bool probe() override;
    void changeConfig();

protected:
    bool fetch_pressure(uint32_t &p, uint16_t &t) override;
    bool offsetPlausible(uint32_t offset) override;
    int  getMaxACOffset() override;

private:
    float getTemperature(void); // returns temperature of last measurement
};
