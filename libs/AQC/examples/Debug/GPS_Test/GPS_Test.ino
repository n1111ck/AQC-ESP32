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
  aqc->updateGPS();
  Serial.print(aqc->getSatellites());
  Serial.print(" ");
  Serial.print(aqc->getLatitude());
  Serial.print(" ");
  Serial.print(aqc->getLongitude());
  Serial.print(" ");
  Serial.println(aqc->getFailed());
}