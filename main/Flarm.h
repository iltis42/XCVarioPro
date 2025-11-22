
#pragma once

#include "Units.h"

class AdaptUGC;
class FlarmMsg;
class GarminMsg;
class GpsMsg;
class FlarmScreen;

class Flarm {
	friend class FlarmMsg;
	friend class GarminMsg;
	friend class GpsMsg;
	friend class FlarmScreen;
public:
	static inline int alarmLevel(){ return AlarmLevel; };
	static inline bool getGPS( float &gndSpeedKmh, float &gndTrack ) {
		if( myGPS_OK ) {
			gndSpeedKmh = Units::knots2kmh(gndSpeedKnots);
			gndTrack = gndCourse;
			return true;
		}
		else{
			return false;
		}
	}
	static inline bool getGPSknots( float &gndSpeed ) {
			if( myGPS_OK ) {
				gndSpeed = gndSpeedKnots;
				return true;
			}
			else{
				return false;
			}
	}
	static inline bool gpsStatus() { return myGPS_OK; }
	static float getGndSpeedKnots() { return gndSpeedKnots; }
	static float getGndCourse() { return gndCourse; }
	static bool validExtAlt() { if( ext_alt_timer ) //fixme -> watchdog
		return true;
	else
		return false;
	}

private:
	static int RX,TX,GPS,Power;
	static int AlarmLevel;
	static int RelativeBearing,RelativeVertical,RelativeDistance;
	static float gndSpeedKnots;
	static float gndCourse;
	static bool   myGPS_OK;
	static int AlarmType;
	static char ID[20];

	static int ext_alt_timer;
	static int _numSat;
	// static int clock_timer;
	// static bool time_sync;
};


