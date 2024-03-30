#include "CReceiver/CReceiver.h"

AQC::CReceiver::CReceiver():
    HardwareSerial(2),
    m_ref()
{

}

AQC::CReceiver::~CReceiver()
{

}

void 
AQC::CReceiver::begin(uint32_t baudrate, uint8_t rxPin)
{
    HardwareSerial::begin(baudrate, SERIAL_8N1, rxPin);

    // Start thread
    xTaskCreatePinnedToCore(
        &CReceiver::thread,             //Function to implement the task 
        "Receiver",                     //Name of the task
        4096,                           //Stack size in words 
        reinterpret_cast<void*>(this),  //Task input parameter 
        1,                              //Priority of the task 
        NULL,                           //Task handle.
        0                               //Core where the task should run 
    );    
}

AQC::SReferenceData 
AQC::CReceiver::getReference()
{
    m_refMutex.lock();
    SReferenceData rValue = m_ref;
    m_refMutex.unlock();

    return rValue;
}

int32_t 
AQC::CReceiver::checkSum(const SReferenceData& ref)
{
    const uint8_t* buffer = reinterpret_cast<const uint8_t*>(&ref);
    uint16_t addBuffer = 0;
    uint8_t result;

    for(uint16_t i = 0; i < sizeof(SReferenceData); i++)
    {
        addBuffer += buffer[i];
    }

    result = static_cast<uint8_t>(addBuffer) + static_cast<uint8_t>(addBuffer >> 8);

    return result == (pow(2,8) - 1) ? 1 : -1;
}


void 
AQC::CReceiver::thread(void* params)
{
    CReceiver* instance = reinterpret_cast<CReceiver*>(params);
    uint8_t* buffer = static_cast<uint8_t*>(malloc(sizeof(SReferenceData)));
    SReferenceData refBuffer;   
    uint8_t byteIndex;
    uint8_t newLineBuffer;

    while(1)
    {
        // Wait break line
        newLineBuffer = 0;
        while(newLineBuffer != 0x0A)
        {
            if(instance->HardwareSerial::available())
            {
                newLineBuffer = instance->HardwareSerial::read();
            }
            vTaskDelay(10);
        }

        // Receiving message
        for(byteIndex = 0; (byteIndex < sizeof(SReferenceData)); byteIndex++)
        {
            while(instance->HardwareSerial::available() == 0)
            {
                vTaskDelay(10);
            };
            buffer[byteIndex] = instance->HardwareSerial::read();
        }     

        // Length of data equals struct size
        if(byteIndex == sizeof(SReferenceData))
        {
            // Cast to reference type
            refBuffer = *reinterpret_cast<SReferenceData*>(buffer);

            // Checksum
            if(checkSum(refBuffer) > 0)
            {
                instance->m_refMutex.lock();
                instance->m_ref = refBuffer;
                instance->m_refMutex.unlock();
            }
        }

        vTaskDelay(10);
    }
}
