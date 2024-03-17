#include "CDeployer/CDeployer.h"

AQC::CDeployer::CDeployer(uint8_t port):
    m_port(port)
{

}

int32_t
AQC::CDeployer::begin()
{
    Servo::attach(m_port);
    arm();
    return 0;
}

void
AQC::CDeployer::arm()
{
    Servo::write(90);
}

void
AQC::CDeployer::deploy()
{
    Servo::write(0);
}

