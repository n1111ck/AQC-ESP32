#include "AQC.h"
#include "Wire.h"
#include <string>
#include <vector>
#include "DataTypes/DataTypes.h"

AQC::CAQCHandler::CAQCHandler(
                float_t timeSample,
                SModel model,
                SMotor motor,
                SController controller
                ):
    m_sensorsHandler(timeSample, 17, 16, 27, 35, 14, 34),
    m_model(model),
    m_motor(motor),
    m_controller(controller)
{

}

int32_t
AQC::CAQCHandler::begin()
{
    
    int32_t beginResult = 0;

    // Start sensors
    beginResult += m_sensorsHandler.begin();
    /*

    // Start filesystem
    beginResult += m_filesystem.begin("final");

    // Start RF Receiver
    m_receiver.begin(2400, 39);

    // Start motors

    // Check for errors
    if(beginResult >= 0)
    {
        // Start controller loop 
    }
    else
    {
        // Save error code
        m_errorCode = beginResult;

        // Start error thread
        xTaskCreatePinnedToCore(
            &CAQCHandler::threadError,              //Function to implement the task 
            "ErrorPrinter",                         //Name of the task
            4096,                                   //Stack size in words 
            reinterpret_cast<void*>(&m_errorCode),  //Task input parameter 
            1,                                      //Priority of the task 
            NULL,                                   //Task handle.
            0                                       //Core where the task should run 
        );  
    }
    */

    return beginResult;
}

AQC::FVector3
AQC::CAQCHandler::getAngle()
{
    return m_sensorsHandler.getAngle();
}

float_t
AQC::CAQCHandler::getAltitude()
{
    return m_sensorsHandler.getAltitude();
}

float_t
AQC::CAQCHandler::getAltitudeVelocity()
{
    return m_sensorsHandler.getAltitudeVelocity();
}

void
AQC::CAQCHandler::refresh()
{
    m_sensorsHandler.refresh();
}

void 
AQC::CAQCHandler::updateGPS()
{
    m_sensorsHandler.updateGPS();
}

double_t
AQC::CAQCHandler::getLatitude()
{
   return  m_sensorsHandler.getLatitude();
}

double_t
AQC::CAQCHandler::getLongitude()
{
    return m_sensorsHandler.getLongitude();
}

uint32_t
AQC::CAQCHandler::getSatellites()
{
    return m_sensorsHandler.getSatellites();
}

uint32_t
AQC::CAQCHandler::getFailed()
{
    return m_sensorsHandler.getFailed();
}

void
AQC::CAQCHandler::addToQueue(std::string message)
{
    m_filesystem.add(message);
}

float_t
AQC::CAQCHandler::getReference()
{
    return m_receiver.getReference().m_altitude;
}

float_t
AQC::CAQCHandler::getDistance()
{
    return m_sensorsHandler.getBelowDistance();
}

void 
AQC::CAQCHandler::threadError(void* params)
{
    int32_t errorCodeRaw = *reinterpret_cast<int32_t*>(params);

    // Set internal led pin
    pinMode(2, OUTPUT);
    digitalWrite(2, 0);

    Serial.println(errorCodeRaw);

    // Parse error code
    bool filesystemError = errorCodeRaw/SErrorCode::s_FILESYSTEM != 0;
    filesystemError ? errorCodeRaw -= SErrorCode::s_FILESYSTEM : 0;

    bool gpsError = errorCodeRaw/SErrorCode::s_GPS != 0;
    gpsError ? errorCodeRaw -= SErrorCode::s_GPS : 0;

    bool compassError = errorCodeRaw/SErrorCode::s_COMPASS != 0;
    compassError ? errorCodeRaw -= SErrorCode::s_COMPASS : 0;

    bool baroError = errorCodeRaw/SErrorCode::s_BARO != 0;
    baroError ? errorCodeRaw -= SErrorCode::s_BARO : 0;

    bool imuError = errorCodeRaw/SErrorCode::s_IMU != 0;
    imuError ? errorCodeRaw -= SErrorCode::s_IMU : 0;
    
    // Error results
    std::vector<bool> errorDetection = {
        imuError, 
        baroError,
        compassError,
        gpsError,
        filesystemError
    };
    bool ledState;
    uint32_t startTime;
    // Start loop
    while(1)
    {
        for(uint8_t errorIt = 0; errorIt < errorDetection.size(); errorIt++)
        {
            if(errorDetection[errorIt])
            {
                ledState = false;
                startTime = millis();
                while(millis() - startTime < 1000)
                {
                    digitalWrite(2, ledState);
                    ledState = !ledState;
                    vTaskDelay(pdMS_TO_TICKS(1000.0/static_cast<float_t>(2*(errorIt + 1))));
                }
                digitalWrite(2, false);

                // Wait 1 second for next error
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        vTaskDelay(10);
    }
}