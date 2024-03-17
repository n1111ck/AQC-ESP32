#ifndef _DATATYPES_H_
#define _DATATYPES_H_

namespace AQC
{
    struct FVector3
    {
        float_t m_x;
        float_t m_y;
        float_t m_z;

        double_t norm()
        {
            return sqrt(pow(m_x, 2) + pow(m_y, 2) + pow(m_z, 2));
        }

        String to_string()
        {
            return String(m_x)+
                " " + String(m_y)+
                " " + String(m_z);
        }

        FVector3& operator-(const FVector3& vector)
        {
            m_x -= vector.m_x;
            m_y -= vector.m_y;
            m_z -= vector.m_z;

            return *this;
        }

        FVector3& operator/(const FVector3& vector)
        {
            m_x += vector.m_x;
            m_y += vector.m_y;
            m_z += vector.m_z;

            return *this;
        }

        FVector3& operator/(const double_t div)
        {
            m_x /= div;
            m_y /= div;
            m_z /= div;

            return *this;
        }

        FVector3& operator/=(const double_t div)
        {
            m_x /= div;
            m_y /= div;
            m_z /= div;

            return *this;
        }

        FVector3& operator*(const double_t mult)
        {
            m_x *= mult;
            m_y *= mult;
            m_z *= mult;

            return *this;
        }

        FVector3& operator*=(const double_t mult)
        {
            m_x *= mult;
            m_y *= mult;
            m_z *= mult;

            return *this;
        }
    };

    struct FStateVector
    {
        float_t m_altitude;
        float_t m_roll;
        float_t m_pitch;
        float_t m_yaw;

        String to_string()
        {
            return String(m_altitude)+
                " " + String(m_roll)+
                " " + String(m_pitch)+ 
                " " + String(m_yaw);
        }

        FStateVector& operator-(const FStateVector& vector)
        {
            m_altitude -= vector.m_altitude;
            m_roll -= vector.m_roll;
            m_pitch -= vector.m_pitch;
            m_yaw -= vector.m_yaw;

            return *this;
        }

        FStateVector& operator/(const FStateVector& vector)
        {
            m_altitude += vector.m_altitude;
            m_roll += vector.m_roll;
            m_pitch += vector.m_pitch;
            m_yaw += vector.m_yaw;

            return *this;
        }

        FStateVector& operator/(const double_t div)
        {
            m_altitude /= div;
            m_roll /= div;
            m_pitch /= div;
            m_yaw /= div;

            return *this;
        }

        FStateVector& operator/=(const double_t div)
        {
            m_altitude /= div;
            m_roll /= div;
            m_pitch /= div;
            m_yaw /= div;

            return *this;
        }

        FStateVector& operator*(const double_t mult)
        {
            m_altitude *= mult;
            m_roll *= mult;
            m_pitch *= mult;
            m_yaw *= mult;

            return *this;
        }

        FStateVector& operator*=(const double_t mult)
        {
            m_altitude *= mult;
            m_roll *= mult;
            m_pitch *= mult;
            m_yaw *= mult;

            return *this;
        }
    };

    struct SReferenceData
    {
        FVector3 m_angles;
        float_t m_altitude;
        uint8_t m_mode;
        uint8_t m_checkSum;
    };

    struct SModel
    {
        float_t m_mass;
        FVector3 m_inertia;
        float_t m_rotationalInertia;
        float_t m_armLength;
    };

    struct SMotor
    {
        float_t m_kv;
        float_t m_b;
        float_t m_d;
    };

    struct SController
    {
        FStateVector m_gainSpeedLoop;
        FStateVector m_gainPositionalLoop;
        FStateVector m_limitSpeed;
    };

    enum EAutoStates
    {
        STDBY, INIT, TAKEOFF, DELIVER, LAND, COLLISION
    };

    struct SErrorCode
    {
        static const int32_t s_IMU = -1;
        static const int32_t s_BARO = -10;
        static const int32_t s_COMPASS = -100;
        static const int32_t s_GPS = -1000;
        static const int32_t s_FILESYSTEM = -10000;
    };

} // namespace AQC


#endif //_DATATYPES_H_