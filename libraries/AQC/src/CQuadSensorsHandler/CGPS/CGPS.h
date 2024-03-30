#ifndef _CGPS_H_
#define _CGPS_H_

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

namespace AQC
{
    class CGPS : TinyGPSPlus
    {
        public:
            CGPS(uint8_t rxPin, uint8_t txPin);

            int32_t begin();
            void refresh();

            uint32_t getFailed();
            double_t getLatitude();
            double_t getLongitude();
            uint32_t getSatellites();
        private:
            HardwareSerial m_serial;

            uint32_t m_numOfSats;
            uint32_t m_checkSum;
            double_t m_latitude;
            double_t m_longitude;

            // Pins
            uint8_t m_rxPin;
            uint8_t m_txPin;

            static const uint32_t s_BAUD_RATE = 9600;
            static const uint32_t s_TIMEOUT_ms = 10000;
    };
}

#endif // _CGPS_H_