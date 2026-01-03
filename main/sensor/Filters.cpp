/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Filters.h"

#include "Atmosphere.h"

#include <cmath>

void LowPassFilter::reset(float init_val)
{
    _last_output = init_val;
}
float LowPassFilter::filter(float input)
{
    _last_output = _alpha * (input - _last_output) + _last_output;
    return _last_output;
}

float AirSpeedFilter::filter(float input)
{
    return Atmosphere::pascal2kmh(fabsf(_lpf.filter(input)));
}
