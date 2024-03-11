#ifndef _AQC_H_
#define _AQC_H_

#include "CQuadSensorsHandler/CQuadSensorsHandler.h"
#include "DataTypes/DataTypes.h"

namespace AQC
{
    class CAQCHandler
    {
        public:
            CAQCHandler();

            FVector3 getAngle();
            float_t getAltitude();
            float_t getAltitudeVelocity();
            void refresh();
        private:
            CQuadSensorsHandler m_sensorsHandler;

    };
} // namespace AQC


#endif // _AQC_H_