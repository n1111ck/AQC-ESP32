#include "CDistanceDetector/CDistanceDetector.h"

AQC::CDistanceDetector::CDistanceDetector(uint8_t trigPin, uint8_t echoPin):
    m_trigPin(trigPin),
    m_echoPin(echoPin)
{
    pinMode(m_trigPin, OUTPUT);
    pinMode(m_echoPin, INPUT);
    s_SENSORS.push_back(this);
}

int32_t 
AQC::CDistanceDetector::begin()
{
    // Start error thread
    xTaskCreatePinnedToCore(
        &CDistanceDetector::threadFunction,     //Function to implement the task 
        "DistanceSensor",                       //Name of the task
        4096,                                   //Stack size in words 
        NULL,                                   //Task input parameter 
        5,                                      //Priority of the task 
        NULL,                                   //Task handle.
        0                                       //Core where the task should run 
    );  

    return 0;
}

void
AQC::CDistanceDetector::threadFunction(void* params)
{
    for(CDistanceDetector* sensor : s_SENSORS)
    {
        // Clears the trigPin
        digitalWrite(sensor->m_trigPin, false);
        ets_delay_us(2);
    }

    uint32_t duration;
    while(1)
    {
        for(CDistanceDetector* sensor : s_SENSORS)
        {
            // Sets the trigPin on HIGH state for 10 micro seconds
            digitalWrite(sensor->m_trigPin, HIGH);
            ets_delay_us(10);
            digitalWrite(sensor->m_trigPin, LOW);

            // Reads the echoPin, returns the sound wave travel time in microseconds
            duration = pulseIn(sensor->m_echoPin, HIGH, 24000);

            // Too close or too away
            if(!(duration > 0.0))
            {
                duration = 24000.0;
            }

            // Save data
            sensor->m_distanceMutex.lock();
            sensor->m_distance = static_cast<float_t>(duration) * 3.4e-4 / 2.0;;
            sensor->m_distanceMutex.unlock();
        }
        vTaskDelay(10);
    }
}

float_t 
AQC::CDistanceDetector::getDistance()
{
    m_distanceMutex.lock();
    float_t distance = m_distance;
    m_distanceMutex.unlock();

    return distance;
}

std::vector<AQC::CDistanceDetector*> AQC::CDistanceDetector::s_SENSORS;