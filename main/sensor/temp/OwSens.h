/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "sensor/SensorBase.h"

#include "../SensorMgr.h"

#include <onewire_types.h>
#include <cstdint>


class OwSens : public SensorTP<float> {
public:
    OwSens() = delete;
    OwSens(onewire_device_address_t addr, SensorId id) : SensorTP<float>(nullptr, 1000, id), _address(addr) {
        _latency_ms = 800;
    };
    virtual uint8_t family() = 0;
    virtual bool primeRead(uint32_t now_ms) = 0;
    onewire_device_address_t getAddress() const { return _address; }
    bool isConverting() const { return _converting; }
    uint32_t getConvertStartMs() const { return _convert_start_ms; }

protected:
    onewire_device_address_t _address;
    bool _converting = false;
    uint32_t _convert_start_ms = 0;
};
