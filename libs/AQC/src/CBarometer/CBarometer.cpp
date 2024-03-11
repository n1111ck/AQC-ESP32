#include "CBarometer/CBarometer.h"

AQC::CBarometer::CBarometer()
{

}

AQC::CBarometer::~CBarometer()
{
    
}

float_t
AQC::CBarometer::getAltitude()
{
    return Adafruit_BMP085::readAltitude() - m_initialAltitude;
}

void
AQC::CBarometer::calibrate()
{
    m_initialAltitude = Adafruit_BMP085::readAltitude();
}

int32_t
AQC::CBarometer::begin(TwoWire* bus)
{
    // Begin Adafruit Sensor
    int32_t rState = Adafruit_BMP085::begin(BMP085_STANDARD, bus) ? 1 : -1;

    // Calibrate sensor to offset current altitude
    calibrate();

    return rState;
}