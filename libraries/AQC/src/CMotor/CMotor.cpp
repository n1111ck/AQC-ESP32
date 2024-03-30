#include "CMotor/CMotor.h"

AQC::CMotor::CMotor(uint8_t port, uint16_t maxPercentage):
    m_port(port),
    m_powerLimitation(static_cast<float_t>(maxPercentage)/100.0)
{

}

int32_t
AQC::CMotor::begin()
{
    Servo::attach(m_port, s_MIN_VALUE, s_MAX_VALUE);
    speed(0.0);
    return 0;
}

void
AQC::CMotor::calibrate()
{
    Servo::attach(m_port, s_MIN_VALUE, s_MAX_VALUE);
    Servo::writeMicroseconds(s_ARM_VALUE);
    vTaskDelay(pdMS_TO_TICKS(5000));
    speed(0.0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    speed(5.0);
}

void
AQC::CMotor::speed(float_t thrustPercentage)
{
    thrustPercentage *= m_powerLimitation;

    Servo::writeMicroseconds(map(thrustPercentage, 0, 100, s_MIN_VALUE, s_MAX_VALUE));
    Serial.println(thrustPercentage);
}