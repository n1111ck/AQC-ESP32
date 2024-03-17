#include "CGPS/CGPS.h"
#include "DataTypes/DataTypes.h"

AQC::CGPS::CGPS(uint8_t rxPin, uint8_t txPin):
    m_serial(1),
    m_latitude(0.0),
    m_longitude(0.0),
    m_numOfSats(0),
    m_checkSum(0),
    m_rxPin(rxPin),
    m_txPin(txPin)
{

}

int32_t
AQC::CGPS::begin()
{
    // Start serial
    m_serial.begin(s_BAUD_RATE, SERIAL_8N1, m_rxPin, m_txPin);

    // Check for connection
    uint32_t time = millis();
    int32_t receivedValid = SErrorCode::s_GPS;
    while(((millis() - time) < s_TIMEOUT_ms) && (receivedValid < 0))
    {
        refresh();
        if(m_numOfSats > 0 || m_checkSum > 0)
        {
            receivedValid = 0;
        }
    }

    return receivedValid;
}

void
AQC::CGPS::refresh()
{
    // Read data
    while (m_serial.available())
    {
        TinyGPSPlus::encode(m_serial.read());
    }

    // Save data on variables
    m_numOfSats = TinyGPSPlus::satellites.value();
    m_latitude = TinyGPSPlus::location.lat();
    m_longitude = TinyGPSPlus::location.lng();
    m_checkSum = TinyGPSPlus::failedChecksum();
}

double_t 
AQC::CGPS::getLatitude()
{
    return m_latitude;
}

double_t 
AQC::CGPS::getLongitude()
{
    return m_longitude;
}

uint32_t 
AQC::CGPS::getSatellites()
{
    return m_numOfSats;
}

uint32_t
AQC::CGPS::getFailed()
{
    return m_checkSum;
}
     