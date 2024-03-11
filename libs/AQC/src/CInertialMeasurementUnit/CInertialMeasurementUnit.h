#ifndef _CINERTIALMEASUREMENTUIT_H_
#define _CINERTIALMEASUREMENTUIT_H_

#include <Adafruit_MPU6050.h>
#include <mutex>
#include "Wire.h"
#include "CISensor/CISensor.h"
#include "DataTypes/DataTypes.h"

namespace AQC
{
    class CInertialMeasurementUnit : Adafruit_MPU6050, CISensor
    {
        public: 
            CInertialMeasurementUnit(FVector3 accelOffset = FVector3({0, 0, 0}));
            ~CInertialMeasurementUnit();

            int32_t begin(TwoWire* bus = &Wire);
            void calibrate();

            void getData(FVector3& angularRate, FVector3& linAccel);
            float_t getGravity();

            FVector3 estimateAnglesFromAccel(FVector3 linAccel);
            float_t getInertialZ(const FVector3& accel, const FVector3& angle);
        private:

            // Threshold values
            FVector3 m_linAccelOffset; 
            FVector3 m_angVelOffset; 

            // Calibrate offsets
            static const uint32_t s_NUM_OF_CYCLES_CALIBRATE = 100;
            static constexpr float_t s_COMMON_GRAVITY = 9.81;
            float_t m_gravity;
    };
}

#endif // _CINERTIALMEASUREMENTUIT_H_