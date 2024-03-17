#ifndef _CFILESYSTEM_H_
#define _CFILESYSTEM_H_

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <string>
#include <deque>
#include <mutex>

// SD Card instance is instantiated inside SD.h class

namespace AQC
{
    class CFilesystem
    {
        public:
            CFilesystem();
            int32_t begin(std::string filename);
            void add(std::string message);
            void refresh();
        private:
            std::string m_filename;

            // Queue variables
            std::deque<std::string> m_queue;
            std::mutex m_queueMutex;

            static const std::string s_COLUMNS;
            static void thread(void * params);
    };
}

#endif // _CFILESYSTEM_H_