#ifndef _AQC_H_
#define _AQC_H_

#include "CQuadSensorsHandler/CQuadSensorsHandler.h"
#include "CFilesystem/CFilesystem.h"
#include "CReceiver/CReceiver.h"
#include "DataTypes/DataTypes.h"
#include <mutex>

namespace AQC
{
    class CAQCHandler
    {
        public:
            CAQCHandler(
                float_t timeSample,
                SModel model,
                SMotor motor,
                SController controller
            );
            CAQCHandler();
            ~CAQCHandler();

            int32_t begin();

            // Temp functions for testing
            FVector3 getAngle();
            float_t getAltitude();
            float_t getAltitudeVelocity();
            void refresh();
            void updateGPS();
            uint32_t getFailed();
            double_t getLatitude();
            double_t getLongitude();
            uint32_t getSatellites();
            void addToQueue(std::string message);
            float_t getReference();
            float_t getDistance();
        private:
            CQuadSensorsHandler m_sensorsHandler;
            CReceiver m_receiver;
            CFilesystem m_filesystem;

            // Input members
            SModel m_model;
            SMotor m_motor;
            SController m_controller;

            // Thread to print led error
            int32_t m_errorCode;
            static void threadError(void* params);

    };

} // namespace AQC

#endif // _AQC_H_