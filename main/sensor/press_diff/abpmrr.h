
#pragma once

#include "AsSensI2c.h"

#include <cstdint>

class ABPMRR final : public AsSensI2c
{
public:
    // instance methods
    ABPMRR();
    ~ABPMRR() = default;

    const char *name() const override { return "ABPMRR"; }
    void changeConfig();

protected:
    bool offsetPlausible(uint32_t offset) override;
    int  getMaxACOffset() override;

private:
    float getTemperature(); // returns temperature of last measurement

    uint16_t t_dat; // 11 bit temperature data
};
