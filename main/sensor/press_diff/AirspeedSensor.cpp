/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "AirspeedSensor.h"

#include "setup/SetupNG.h"
#include "logdef.h"

#include <freertos/FreeRTOS.h>

static float as_buffer[ (SENSOR_HISTORY_DURATION_MS / 100) + 4 ]; // history buffer for airspeed sensor

AirspeedSensor::AirspeedSensor() : SensorTP<float>(as_buffer, 100)
{
}

bool AirspeedSensor::setup()
{
    ESP_LOGI(FNAME, "Airspeed sensor %s", name());

    _offset = as_offset.get();
    if (_offset < 0) {
        ESP_LOGI(FNAME, "offset not yet done: need to recalibrate");
    }
    else {
        ESP_LOGI(FNAME, "offset from NVS: %0.1f", _offset);
    }

    uint32_t adcval = 0;
    uint16_t temp;
    fetch_pressure(adcval, temp);

    ESP_LOGI(FNAME, "offset from ADC %u", (unsigned)adcval);

    bool plausible = offsetPlausible(adcval);
    if (plausible) {
        ESP_LOGI(FNAME, "offset from ADC is plausible");
    }
    else {
        ESP_LOGI(FNAME, "offset from ADC is NOT plausible");
    }

    int deviation = std::abs(_offset - adcval);
    if (deviation < getMaxACOffset())
    {
        ESP_LOGI(FNAME, "Deviation in bounds");
    }
    else
    {
        ESP_LOGI(FNAME, "Deviation out of bounds");
    }

    // Long term stability of Sensor as from datasheet 0.5% per year -> 4000 * 0.005 = 20
    if (_offset < 0 || (plausible && deviation < getMaxACOffset()) || autozero.get())
    {
        ESP_LOGI(FNAME, "Airspeed OFFSET correction ongoing, calculate new _offset...");
        if (autozero.get())
            autozero.set(0);
        uint32_t rawOffset = 0;
        for (int i = 0; i < 100; i++)
        {
            fetch_pressure(adcval, temp);
            rawOffset += adcval;
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        _offset = std::roundf(rawOffset / 100.f);
        if (offsetPlausible(_offset))
        {
            ESP_LOGI(FNAME, "Offset procedure finished, offset: %f", _offset);
            as_offset.set(_offset);
        }
        else
        {
            ESP_LOGW(FNAME, "Offset out of tolerance, ignore odd offset value");
        }
    }
    else
    {
        ESP_LOGI(FNAME, "No new Calibration: flying with plausible pressure");
    }
    return true;
}

float AirspeedSensor::doRead()
{
    uint32_t p_raw;
    uint16_t t_dat;
    bool ok = fetch_pressure(p_raw, t_dat);
    if (!ok)
    {
        ESP_LOGE(FNAME, "Retry measure, status :%d  p=%u", ok, (unsigned)p_raw);
        ok = fetch_pressure(p_raw, t_dat);
        if (!ok)
        {
            ESP_LOGE(FNAME, "Warning, status :%d  p=%u, bad even retry", ok, (unsigned)p_raw);
            return NAN;
        }
    }
    float _pascal = (p_raw - _offset) * _multiplier;
    ESP_LOGI(FNAME,"pressure: %f offset: %d raw: %u  raw-off:%f m:%f", _pascal, (int)_offset, (unsigned)p_raw,  (_offset - p_raw),  _multiplier );
    return _pascal;
}
