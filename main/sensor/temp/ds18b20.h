/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "sensor/temp/OwSens.h"

struct onewire_bus_t;

class DS18B20 final : public OwSens {
public:
    DS18B20(onewire_device_address_t addr) : OwSens(addr) {}
    virtual ~DS18B20() = default;
    const char* name() const override { return "DS18B20"; }
    bool probe() override { return false; };
    bool setup() override;
    float doRead();

    // OW
    uint8_t family() override { return 0x28; }
    bool primeRead(uint32_t now_ms) override;

};
