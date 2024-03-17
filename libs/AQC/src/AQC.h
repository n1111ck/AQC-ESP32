#ifndef _AQC_H_
#define _AQC_H_

#include "CQuadSensorsHandler/CQuadSensorsHandler.h"
#include "CFilesystem/CFilesystem.h"
#include "CReceiver/CReceiver.h"
#include "DataTypes/DataTypes.h"
#include "CMotor/CMotor.h"
#include "CDeployer/CDeployer.h"
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

            int32_t begin();
            void calibrate();
        private:
            // Sensors
            CQuadSensorsHandler m_sensorsHandler;

            // Receiver
            CReceiver m_receiver;

            // Filesystem
            CFilesystem m_filesystem;

            // Input members
            SModel m_model;
            SMotor m_motor;
            SController m_controller;

            // Motors
            CMotor m_pMotor[4];

            // Deployer
            CDeployer m_deployer;

            // Thread to print led error
            int32_t m_errorCode;
            static void threadError(void* params);
    };

} // namespace AQC

#endif // _AQC_H_