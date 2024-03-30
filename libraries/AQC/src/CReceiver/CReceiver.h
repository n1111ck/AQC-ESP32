#ifndef _CRECEIVER_H_
#define _CRECEIVER_H_

#include <HardwareSerial.h>
#include "DataTypes/DataTypes.h"
#include <mutex>

namespace AQC
{
    class CReceiver : HardwareSerial
    {
        public:
            CReceiver();
            ~CReceiver();
            void begin(uint32_t baudrate, uint8_t rxPin);

            SReferenceData getReference();
        private:
            // Reference variables
            std::mutex m_refMutex;
            SReferenceData m_ref;

            // Checksum function
            static int32_t checkSum(const SReferenceData& ref);

            // Thread function
            static void thread(void* params);
    };
}

#endif // _CRECEIVER_H_
