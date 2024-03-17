#include "CCompass.h"

AQC::CCompass::CCompass()
{

}

AQC::CCompass::~CCompass()
{
    
}

void
AQC::CCompass::calibrate()
{
    // calibration code to declination angle
    m_declinationAngle = 0.3;
}

float_t
AQC::CCompass::getHeading(const FVector3& angle)
{
    sensors_event_t event; 

    Adafruit_HMC5883_Unified::getEvent(&event);

    return atan2(event.magnetic.y*cos(angle.m_x), event.magnetic.x*cos(angle.m_y)) + m_declinationAngle;
}

int32_t 
AQC::CCompass::begin(TwoWire* bus)
{
    // Begin Adafruit Sensor
    int32_t rState = Adafruit_HMC5883_Unified::begin(bus) ? 0 : SErrorCode::s_COMPASS;

    // TODO: calibration
    calibrate();

    return rState;
}