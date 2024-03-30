#ifndef _CTESTER_H_
#define _CTESTER_H_

#include "CQuadSensorsHandler/CQuadSensorsHandler.h"
#include "CFilesystem/CFilesystem.h"
#include "CReceiver/CReceiver.h"
#include "DataTypes/DataTypes.h"
#include "CMotor/CMotor.h"
#include "CDeployer/CDeployer.h"
#include <mutex>

namespace AQC
{
    class CTester
    {
        public:
            CTester();

            int32_t begin();
            void testMotor(uint8_t id, float_t thurstPercentage);
        private:
            // Motors
            CMotor m_pMotor[4];
    };
}

#endif