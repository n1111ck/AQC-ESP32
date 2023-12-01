#include "AQC_Altimeter.h"

AQC::CAltimeter::CAltimeter(TwoWire* i2c)
  :Adafruit_BMP085(),
  m_numMeasurements(20)
{
  // Init BMP180 sensor
  begin(0, i2c);

  // Capture initial altitude
  m_initialAltitude = readAltitude();

  // Spawn task
  xTaskCreatePinnedToCore (
    interrupt,                        // task function
    "AltimeterTask",                  // Task name
    2048,                             // Stack size
    static_cast<void *>(this),        // Parameters
    0,                                // Priority
    NULL,                             // Task handle.
    0                                 // Core identifier
  );
}

AQC::CAltimeter::~CAltimeter()
{
  
}

void AQC::CAltimeter::update()
{
  m_currentAltitude = readAltitude();
}

float AQC::CAltimeter::getAltitude()
{
  return m_currentAltitude;
}

float AQC::CAltimeter::getInitialAltitude()
{
  return m_initialAltitude;
}

float AQC::CAltimeter::getComparedAltitude()
{
  return m_currentAltitude - m_initialAltitude;
}

static void interrupt(void* altimeter_void)
{
  AQC::CAltimeter* altimeter = static_cast<AQC::CAltimeter*>(altimeter_void);

  while(1)
  {
    altimeter->update();
  }
}