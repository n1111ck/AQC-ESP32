#include "AQC.h"
#include "Wire.h"
#include <string>
#include <vector>
#include "DataTypes/DataTypes.h"

#include <EEPROM.h>

AQC::CAQCHandler::CAQCHandler(
                float_t timeSample,
                SModel model,
                SMotor motor,
                SController controller
                ):
    m_sensorsHandler(timeSample, 16, 17, 27, 35, 14, 34),
    m_model(model),
    m_motor(motor),
    m_controller(controller),
    m_pMotor({ CMotor(32), CMotor(33), CMotor(25), CMotor(26) }),
    m_deployer(13)
{

}

int32_t
AQC::CAQCHandler::begin()
{
    int32_t beginResult = 0;

    // Start motors
    m_pMotor[0].begin();
    m_pMotor[1].begin();
    m_pMotor[2].begin();
    m_pMotor[3].begin();
    
    // Save time (ESC's must wait 2 seconds to arm)
    uint16_t armTime = millis();

    // Start deployer
    m_deployer.begin();

    // Start sensors
    beginResult += m_sensorsHandler.begin();

    // Start filesystem
    beginResult += m_filesystem.begin("final");

    // Start RF Receiver
    m_receiver.begin(2400, 39); 

    // Check for errors
    if(beginResult >= 0)
    {
        // Wait fininsh of arm time
        if(millis() - armTime < 2000)
        {
            vTaskDelay(pdMS_TO_TICKS(5000 - (millis() - armTime)));
        }

        // Start controller loop
        while(1)
        {
            m_sensorsHandler.refresh();
        }
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

    return beginResult;
}

void 
AQC::CAQCHandler::calibrate()
{
    pinMode(2, OUTPUT);
    digitalWrite(2, false);
    m_pMotor[0].calibrate();
    m_pMotor[1].calibrate();
    m_pMotor[2].calibrate();
    m_pMotor[3].calibrate();
    digitalWrite(2, true);
}

void 
AQC::CAQCHandler::threadError(void* params)
{
    int32_t errorCodeRaw = *reinterpret_cast<int32_t*>(params);

    // Set internal led pin
    pinMode(2, OUTPUT);
    digitalWrite(2, 0);

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