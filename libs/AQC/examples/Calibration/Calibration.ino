#include "AQC.h"

AQC::CAQCHandler* aqc;

void setup()
{
    AQC::SModel model;
    AQC::SMotor motor;
    AQC::SController controller;
    aqc = new AQC::CAQCHandler(30e-3, model, motor, controller);
    aqc->calibrate();
}

void loop()
{

}