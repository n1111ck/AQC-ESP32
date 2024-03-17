#ifndef _CBAROMETER_H_
#define _CBAROMETER_H_

#include <Adafruit_BMP085.h>
#include "Wire.h"
#include "CQuadSensorsHandler/CISensor/CISensor.h"

namespace AQC
{
    class CBarometer : Adafruit_BMP085, CISensor 
    {
        public:
            CBarometer();
            ~CBarometer();

            int32_t begin(TwoWire* bus = &Wire);
            void calibrate();

            float_t getAltitude();
        private:
            // Linear accel variables
            float_t m_initialAltitude;
    };
}

#endif //_CBAROMETER_H_