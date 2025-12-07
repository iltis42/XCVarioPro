/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "OneWireBus.h"

#include "logdef.h"

#include <onewire_bus.h>
#include <onewire_bus_interface.h>

#include <driver/gpio.h>

OneWireBus *OneWIRE = nullptr;

const int OneWireBus::_ONEWIRE_BUS_GPIO = GPIO_NUM_23;

OneWireBus::OneWireBus() : InterfaceCtrl(false, false)
{
}

OneWireBus* OneWireBus::create()
{
    if ( ! OneWIRE ) {
        OneWIRE = new OneWireBus();
    }
    return OneWIRE;
}

OneWireBus::~OneWireBus()
{
    OneWIRE = nullptr;
    if ( _onewire) {
        // onewire_bus_del( (onewire_bus_handle_t)_onewire );
        _onewire->del(_onewire);
    }
}

void OneWireBus::ConfigureIntf(int cfg)
{
        // install new 1-wire bus
    onewire_bus_handle_t bus;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = _ONEWIRE_BUS_GPIO,
        .flags = {
            .en_pull_up = false,
        }
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI(FNAME, "1-Wire bus installed on GPIO%d", _ONEWIRE_BUS_GPIO);

    _onewire = bus;
    OneWIRE = this;
}
