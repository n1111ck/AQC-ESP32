#include "CTester.h"

AQC::CTester::CTester():
    m_pMotor({ CMotor(32), CMotor(33), CMotor(25), CMotor(26) })
{

}

int32_t
AQC::CTester::begin()
{
    // Start motors
    m_pMotor[0].begin();
    m_pMotor[1].begin();
    m_pMotor[2].begin();
    m_pMotor[3].begin();

    // Save time (ESC's must wait 2 seconds to arm)
    uint16_t armTime = millis();

    // Wait fininsh of arm time
    if(millis() - armTime < 2000)
    {
        vTaskDelay(pdMS_TO_TICKS(5000 - (millis() - armTime)));
    }

    return 0;
}

void
AQC::CTester::testMotor(uint8_t id, float_t thurstPercentage)
{
    m_pMotor[id].speed(thurstPercentage);
}