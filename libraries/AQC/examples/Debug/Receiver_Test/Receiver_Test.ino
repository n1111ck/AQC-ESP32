#include <AQC.h>

AQC::CAQCHandler* aqc;

void setup()
{
    Serial.begin(115200);
    // put your setup code here, to run once:
    aqc = new AQC::CAQCHandler();
}

void loop()
{
    Serial.println(aqc->getReference());
}