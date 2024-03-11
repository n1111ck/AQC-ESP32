#ifndef _CISENSOR_H_
#define _CISENSOR_H_

namespace AQC
{
    class CISensor
    {
        public:
            virtual int32_t begin(TwoWire* bus) = 0;
            virtual void calibrate() = 0;
    };
}

#endif //_CISENSOR_H_