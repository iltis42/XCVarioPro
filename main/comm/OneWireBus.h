/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "InterfaceCtrl.h"

struct onewire_bus_t;

class OneWireBus final : public InterfaceCtrl
{
private:
    OneWireBus();
public:
    static OneWireBus* create();
    ~OneWireBus();

    // Ctrl
    InterfaceId getId() const override { return OW_BUS; }
    const char *getStringId() const override { return "OneWire"; }
    void ConfigureIntf(int cfg) override;
    int Send(const char*, int&, int) override { return 0; }
    // OneWire specific
    onewire_bus_t *getOneWire() const { return _onewire; }

private:
    onewire_bus_t *_onewire = nullptr;
    static const int _ONEWIRE_BUS_GPIO;
};

extern OneWireBus *OneWIRE;