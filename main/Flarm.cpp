#include "Flarm.h"

// #include <sys/time.h>

int Flarm::RX = 0;
int Flarm::TX = 0;
int Flarm::GPS = 0;
int Flarm::Power = 0;
int Flarm::AlarmLevel = 0;
int Flarm::RelativeBearing = 0;
int Flarm::AlarmType = 0;
int Flarm::RelativeVertical = 0;
int Flarm::RelativeDistance = 0;
float Flarm::gndSpeedKnots = 0;
float Flarm::gndCourse = 0;
bool Flarm::myGPS_OK = false;
char Flarm::ID[20] = "";

int Flarm::ext_alt_timer=0;
int Flarm::_numSat=0;
// int Flarm::clock_timer=0;
// bool Flarm::time_sync=false;



// void Flarm::progress(){  // once per second
// 	// ESP_LOGI(FNAME,"progress, timeout=%d", timeout );
// 	clock_timer++;
// 	if( !(clock_timer%3600) ){  // every hour reset sync flag to wait for next valid GPS time
// 		time_sync = false; //fixme
// 	}
// }


