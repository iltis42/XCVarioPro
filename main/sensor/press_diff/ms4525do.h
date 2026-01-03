
#pragma once


#include "AsSensI2c.h"

#include <cstdint>

class MS4525DO final : public AsSensI2c
{
public:
    // instance methods
    MS4525DO();
    virtual ~MS4525DO() = default;

    const char *name() const override;
    void changeConfig();
    void setAbpmrr() { _is_abpmrr = true; }

protected:
    bool offsetPlausible(int32_t offset) override;
    int getMaxACOffset() override;

private:
    float getTemperature(); // returns temperature of last measurement
    uint16_t t_dat; // 11 bit temperature data
    bool _is_abpmrr = false;
};
