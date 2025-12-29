/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Filters.h"

#include <cmath>

void LowPassFilter::reset(float init_val)
{
    _last_output = init_val;
}
float LowPassFilter::filter(float input)
{
    if ( std::isnan(input) ) {
        return _last_output; // do not update on NaN input
    }
    _last_output = _alpha * (input - _last_output) + _last_output;
    return _last_output;
}
