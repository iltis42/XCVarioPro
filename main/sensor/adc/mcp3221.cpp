
#include "mcp3221.h"

#include "logdefnone.h"

#include <I2Cbus.hpp>

#define MCP3221_CONVERSE 0x4d   //10011010 NOTE IF IT ENDS IN 1, this is the READ ADDRESS. This is all this device does.
                                //It opens a conversation via this specific READ address

// Library for the MCP3221 12 BIT ADC.
//   MCP3221  Top View

//   +  -o   o- SCL
//   -  -o
//   S  -o   o- SDA


MCP3221::MCP3221(i2cbus::I2C *b) : _bus(b), _address(MCP3221_CONVERSE)
{
    // exponential_average = 0;
}

// scan bus for I2C address
esp_err_t MCP3221::selfTest(){
	uint8_t data[2];
	esp_err_t err = _bus->readBytes(MCP3221_CONVERSE, 0, 2, data );
	if( err != ESP_OK ){
		ESP_LOGI(FNAME,"MCP3221 selftest, scan for I2C address %02x FAILED", MCP3221_CONVERSE );
		return ESP_FAIL;
	}
	ESP_LOGI(FNAME,"MCP3221 selftest, scan for I2C address %02x PASSED", MCP3221_CONVERSE );
	return ESP_OK;
}

// float MCP3221::readAVG( float alpha ) {

// 	uint16_t newval;
// 	esp_err_t ret = readRaw(newval);
// 	// ESP_LOGI(FNAME,"Airspeed AD1: %d", newval );
// 	if( ret == ESP_OK ){
// 		// ESP_LOGI(FNAME, "%d", newval );
// 		if ( exponential_average == 0 ){
// 			exponential_average = newval;
// 		}
// 		exponential_average = exponential_average + alpha*(newval - exponential_average);
// 		return exponential_average;
// 	}
// 	else
// 		return 0.0;
// }


// You cannot write to an MCP3221, it has no writable registers.
// MCP3221 also requires an ACKnowledge between each byte sent, before it will send the next byte. So we need to be a bit manual with how we talk to it.
// It also needs an (NOT) ACKnowledge after the second byte or it will keep sending bytes (continuous sampling)
// 
// From the datasheet.
// 
// I2C.START
// Send 8 bit device/ part address to open conversation.   (See .h file for part explanation)
// read a byte (with ACK)
// read a byte (with NAK)
// I2C.STOP

int MCP3221::readVal()
{
    int retval = 0;
    int samples = 0;
    uint16_t as_last = 0;
    for (int i = 0; i < 8; i++)
    {
        uint16_t as;
        if (readRaw(as) == ESP_OK)
        {
            if ( as_last == 0 ) as_last = as;
            if (abs(as - as_last) > 1000) {
                ESP_LOGE(FNAME, "REREAD AS delta OOB dropped, cur:%04x  last:%04x", as, as_last);
            }
            else {
                retval += as;
                samples++;
            }
            as_last = as;
        }
        else {
            ESP_LOGE(FNAME, "Airspeed I2C read error");
        }
    }
    if (samples)
    {
        return retval / samples;
    }
    return 0;
}

esp_err_t MCP3221::readRaw(uint16_t &val)
{
	uint8_t data[2];
	esp_err_t err = _bus->readBytes(MCP3221_CONVERSE, 0, 2, data );
	if( err != ESP_OK ){
		val = 0;
	}
	else{
		val = (data[0] << 8) + data[1];
	}

	return( err );
}


