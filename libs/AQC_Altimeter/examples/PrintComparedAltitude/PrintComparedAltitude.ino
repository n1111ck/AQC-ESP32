#include <AQC_Altimeter.h>
#include <Wire.h>

TwoWire i2c_1 = TwoWire(0);
AQC::CAltimeter* altimeter;

void setup() 
{
  Serial.begin(115200);
  i2c_1.begin(4, 15);
  altimeter = new AQC::CAltimeter(&i2c_1);
}

void loop() {
  Serial.println(altimeter->getComparedAltitude());
}
