#ifndef _CCOMPASS_H_
#define _CCOMPASS_H_

#include <Adafruit_HMC5883_U.h>
#include "CQuadSensorsHandler/CISensor/CISensor.h"
#include "DataTypes/DataTypes.h"
#include "Wire.h"

namespace AQC
{
    class CCompass : Adafruit_HMC5883_Unified, CISensor
    {
        public:
            CCompass();
            ~CCompass();

            int32_t begin(TwoWire* bus);
            void calibrate();

            float_t getHeading(const FVector3& angle);
        private:
            float_t m_declinationAngle;

    };
}

#endif // _CCOMPASS_H_