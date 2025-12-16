# XCVario Pro
An ESP32 based lean variometer system with open data interface featuring OpenVario, Cambridge, Borgelt, Naviter and XCVario format, running on dedicated XCVario hardware.

![Vario](https://github.com/hjr/XCVarioPro/blob/master/XcvPro.png)

The project is rooted in the open source and open hardware flight computer scene using the ESP32 SOC supporting modern digital sensors such as BMP280, SPL06-007, DS1820b, TE4525, ABPMRR, MP5004DP, CAN, AHRS MPU6050, and QMC5883L magnet sensor.

The variometer has a sunlight readable 1000 nits 2.4 inch IPS technology LCD display features a low power consumation of less that 1.2W, that equals to 100mA at 12.5V operating voltage. The size is 64(H)x64(W)x35(D) mm, for a standard 57mm(2") instrument gauge. It weighs only about 170 gramm.

##Vision

While the amazing availability of high-fidelity sensors combined with today’s highly miniaturized computational power provides the perfect ingredients for a project like this, it quickly becomes clear that the overall quality of the resulting system design is tightly linked to the quality of the software it is built on. Recognizing this, it also becomes evident that further improvements in quality and usability of a variometer like this are hardly achievable with a “pick-it-for-free” mentality. Significant effort is required, and that effort needs to be funded — which in turn becomes a major challenge for the project.

The evolution from **XCVario** to **XCVario Pro** therefore comes with the addition of a dedicated software expert to the XCVario team, responsible for all software-related aspects. The variometer will retain a basic feature set based on an open-source code base, as it did initially. In addition, it will offer licensed features intended to fund ongoing maintenance and further development, with the goal of evolving the variometer into a best-in-class product.

The original ambition of providing a high-quality variometer sensor that turns any club glider with an outdated variometer into a fun-to-fly aircraft remains unchanged. Beyond this, several new capabilities will be introduced, addressing key limitations found in common variometer designs that currently prevent optimal performance, along with a number of new features:

* Creation of a synchronized sensor data history for future filter design
* Easy-to-perform high-precision IMU reference calibration
* Continuous gyro drift tracking
* “imuTEK” total energy compensation of the vario signal, with awareness of horizontal wind gusts
*  - automated and continuous calibration of the compensation
*  - no requirement for a TE probe
* Instant wind calculation

## Some Demo Clips
(The material needs a decent update though)

* [Quick Demo on YouTube](https://www.youtube.com/watch?v=Piu5SiNPaRg)
* [Vario Sensitivity Test](https://www.youtube.com/watch?v=RqFLOQ9wvgY)
* [Basic Function Demo](https://www.youtube.com/watch?v=zGldyS57ZgQ)
* [Full Sunlight test](https://www.youtube.com/watch?v=TFL9i2DBNpA)
* [Audio Demo in Flight](https://www.youtube.com/watch?v=6Vc6OHcO_T4)  ("Single Tone" Modus)
* [Basic Vario Setup MC, QNH, Ballast](https://www.youtube.com/watch?v=DvqhuaVlfEI)
* [Airfield Elevation and Vario Setup](https://www.youtube.com/watch?v=x3UIpL9qGec)
* [Bluetooth, Audio and Polar Setup](https://www.youtube.com/watch?v=9HcsfyLX-wE)
* [Flap (WK) Indicator Setup](https://www.youtube.com/watch?v=tP2a2aDoOsg)
* [Vario System Setup](https://www.youtube.com/watch?v=BCR16WUTwJY)
* [Setup XCSoar to connect with Vario](https://www.youtube.com/watch?v=LDgnvLoTekU&t=95s)

##Supported sensors
* TE variometer (TE)
* Airspeed (PI)
* Barometric altitude (BA)
* Outside air temperature
* Battery voltage
* Acceleration and rotation IMU
* Optional earth magnetic field sensor
* Optional flap sensor
* Optional gear warn sensor
* Optional GPS NMEA data stream
* Optional Flarm traffic NMEA data stream
* Optional external instant wind sensors (Anemoi)

The ESP32 module contains a Bluetooth and Wifi  module plus serial interfaces, so we are able to transmit wireless or wired data to any navigation device running XCSoar, LK8000, Naviter and more in various formats as there is the native XCVario, Borgelt, Cambridge and Openvario format,so devices can operate as full glide computer with TE-vario, barometric altitude, speed and more.

##The manual

[Online Handbook in various languages](https://xcvario.com/docs-category/vario)

##A (soft-) feature list
- Variometer display with adjustable range (1 m/s - 30 m/s), and damping ( 1s - 10s )
- QNH, Ballast, Bugs and McCready adjustment and many other options configurable
- Vario sound generator with adjustable Volume and Deadband plus setup option for tone style profiles
- Sound-Sequencer with a variety of sound-bites as sonification to most of the visual variometer information
- Integrated loudspeaker, 2 Watt audio power
- S2F (Speed2Fly) indicator with configurable MC, ballast and bugs based on glider polar
- Around 130 predefined polars are included ( list in https://github.com/hjr/XCVarioPro/blob/master/components/glider/PolarTable.txt )
- IAS or TAS airspeed indication
- OAT (Outside Air Temperature) sensor
- Flap assitent with or w/o flap position sensor
- Battery voltage and status indicator
- Thermal assistent indicator for efficient thermal centering
- Wind rose indicator with optional second, instant wind pointer
- Bluetooth, Wifi and two serial interfaces to external devices (XCSoar, LK8000, FLARM, Anemoi, other sensors ...)
- Complete support of a secondary variometer (two seater) with automatic routing of all connected devices
- High precision barometric altimeter 1 hPa (8 meter) absolute accuracy, and 0.1 meter resolution
- Sunlight readable high contrast 1000 nits 2.4 inch IPS Technology Display
- External switch for S2F/Vario mode
- Setup menu to customize vario features
- Connectivity for the FLARM View display
- Output of NMEA data in OpenVario, Borgelt, Cambridge, Naviter or XCVario format
- OTA (Over The Air) software update via WiFi and web browser
- Attitude and Heading Reference System (AHRS) available since 2021 hardware revision
- Wind calculation in circling and in straight flight (needs a connected GPS source)
- Electronic TE compensation circumventing long TE tubing and TE probe insufficiencies

