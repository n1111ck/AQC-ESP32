#ifndef _CDEPLOYER_H_
#define _CDEPLOYER_H_

#include <ESP32Servo.h>

namespace AQC
{
    class CDeployer : Servo
    {
        public:
            CDeployer(uint8_t port);
            int32_t begin();
            void deploy();
            void arm();
        private:
            uint8_t m_port;
    };
}

#endif // _CDEPLOYER_H_