#include "CFilesystem/CFilesystem.h"
#include "DataTypes/DataTypes.h"

AQC::CFilesystem::CFilesystem()
{
    
}

int32_t
AQC::CFilesystem::begin(std::string filename)
{
    int32_t rStatus = SD.begin() == 1 ? 0 : SErrorCode::s_FILESYSTEM;

    if(rStatus > 0)
    {
        // Start thread
        xTaskCreatePinnedToCore(
            &CFilesystem::thread,           //Function to implement the task 
            "Filesystem",                   //Name of the task
            4096,                           //Stack size in words 
            reinterpret_cast<void*>(this),  //Task input parameter 
            15,                             //Priority of the task 
            NULL,                           //Task handle.
            0                               //Core where the task should run 
        );         

        // Create file
        m_filename = ("/" + filename + ".csv").c_str();
        fs::File file = SD.open(m_filename.c_str(), FILE_WRITE);
        file.print(s_COLUMNS.c_str());
        file.close();     
    }

    return rStatus;
}

void 
AQC::CFilesystem::add(std::string message)
{
    m_queueMutex.lock();
    // Add new message to queue
    m_queue.push_back(message + "\n");
    m_queueMutex.unlock();
}

void
AQC::CFilesystem::refresh()
{
    if(m_queue.size() > 0)
    {
        // Get first value
        fs::File file = SD.open(m_filename.c_str(), FILE_APPEND);
        file.print(m_queue[0].c_str());
        file.close();

        // Pop first element
        m_queue.pop_front();
    }
}

void
AQC::CFilesystem::thread(void* params)
{
    CFilesystem* instance = reinterpret_cast<CFilesystem*>(params);

    while(1)
    {
        instance->m_queueMutex.lock();
        instance->refresh();
        instance->m_queueMutex.unlock();
        vTaskDelay(10);
    }
}

const std::string AQC::CFilesystem::s_COLUMNS = "Time[s],Lattitude[deg],Longitude[deg],Altitude[m],Roll[deg],Pitch[deg],Yaw[deg],AltitudeVelocity[m/s],RollVelocity[deg/s],PitchVelocity[deg/s],YawVelocity[deg/s],M1_speed[V],M2_speed[V],M3_speed[V],M4_speed[V]\n";