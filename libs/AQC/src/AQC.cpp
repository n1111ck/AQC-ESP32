#include "AQC.h"
#include "Wire.h"

AQC::CAQCHandler::CAQCHandler():
    m_sensorsHandler(30e-3)
{

    int32_t beginResult = m_sensorsHandler.begin();

    if(beginResult < 0)
    {
        Serial.print("Error starting sensors code: ");
        Serial.println(beginResult);
        while(1);
    }
}

AQC::FVector3
AQC::CAQCHandler::getAngle()
{
    return m_sensorsHandler.getAngle();
}

float_t
AQC::CAQCHandler::getAltitude()
{
    return m_sensorsHandler.getAltitude();
}

float_t
AQC::CAQCHandler::getAltitudeVelocity()
{
    return m_sensorsHandler.getAltitudeVelocity();
}

void
AQC::CAQCHandler::refresh()
{
    m_sensorsHandler.refresh();
}