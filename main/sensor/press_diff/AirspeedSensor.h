#pragma once

#include "../SensorBase.h"

class AirspeedSensor : public SensorTP<float> {
public:
	AirspeedSensor();
	virtual ~AirspeedSensor() {};

	bool  setup() override;
	float doRead() override;
	virtual void  changeConfig() = 0;

protected:
	virtual bool fetch_pressure(uint32_t &p, uint16_t &t) = 0;
	virtual bool offsetPlausible(uint32_t offset ) = 0;
	virtual int getMaxACOffset() = 0;
	float _offset = 0.; // raw adc offset value (float because of nvs storage)
	float _multiplier = 1.0f;
};

