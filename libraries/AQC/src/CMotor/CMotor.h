#ifndef _CMOTOR_H_
#define _CMOTOR_H_

#include <ESP32Servo.h>

namespace AQC
{
    class CMotor : Servo
    {
        public:
            CMotor(uint8_t port, uint16_t maxPercentage = 35);
            int32_t begin();
            void speed(float_t thrustPercentage);
            void calibrate();
        private:
            uint8_t m_port;
            float_t m_powerLimitation;

            static const uint16_t s_MIN_VALUE = 1000;
            static const uint16_t s_MAX_VALUE = 2000;
            static const uint16_t s_ARM_VALUE = 500;
    };
}

#endif // _CMOTOR_H_
