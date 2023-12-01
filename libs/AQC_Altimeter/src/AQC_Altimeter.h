#ifndef AQC_ALTIMETER_H
#define AQC_ALTIMETER_H

// Extern libraries
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "Arduino.h"


namespace AQC
{
  class CAltimeter : Adafruit_BMP085
  {
    public:
      CAltimeter(TwoWire* i2c);
      ~CAltimeter();

      // Methods
      void update();
      float getAltitude();
      float getInitialAltitude();
      float getComparedAltitude();
    private:
      float m_initialAltitude;
      float m_currentAltitude;
      float m_numMeasurements;
  };
}

// Task
static void interrupt(void* altimeter_void);

#endif // AQC_ALTIMETER_H