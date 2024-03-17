#ifndef _CDISTANCEDETECTOR_H
#define _CDISTANCEDETECTOR_H

#include "Arduino.h"
#include <vector>
#include <mutex>

namespace AQC
{
    class CDistanceDetector
    {
        public:
            CDistanceDetector(uint8_t trigPin, uint8_t echoPin);

            static int32_t begin();
            float_t getDistance();

        private:
            uint8_t m_trigPin;
            uint8_t m_echoPin;

            // Distance variables
            std::mutex m_distanceMutex;
            float_t m_distance;

            static std::vector<CDistanceDetector*> s_SENSORS;
            static void threadFunction(void* params);

    };
}

#endif // _CDISTANCEDETECTOR_H