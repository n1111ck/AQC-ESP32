#include "AQC.h"


AQC::CAQCHandler* aqc;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  aqc = new AQC::CAQCHandler();
}

uint32_t tempo;
uint32_t lastChecked = 0;

void loop() {
  tempo = millis();

  aqc->refresh();

  AQC::FVector3 angle = aqc->getAngle();
  angle *= static_cast<double_t>(180.0/PI);
  
  Serial.print(angle.to_string());

  float_t altitude = aqc->getAltitude();
  float_t altitudeVel = aqc->getAltitudeVelocity();
  Serial.print(" ");
  Serial.print(altitude);
  Serial.print(" ");
  Serial.println(altitudeVel);
  

  if(30e3 - tempo > 0)
  {
    delayMicroseconds(30e3 - tempo);
  }
  else
  {
    Serial.println("Overrun");
  }
}
