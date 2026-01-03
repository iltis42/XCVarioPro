/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/


#include "SensorBase.h"

#include "SensorMgr.h"

SensorBase::SensorBase(int ums, SensorId id) : _update_interval_ms(ums), _latency_ms(0), _last_update_time_ms(0)
{
    SensorRegistry::registerSensor(id, this);
}

SensorBase::~SensorBase()
{
    // deregister
    SensorRegistry::deregisterSensor(this);
}