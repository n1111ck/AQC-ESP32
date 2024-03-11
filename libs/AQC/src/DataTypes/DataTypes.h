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
} // namespace AQC


#endif //_DATATYPES_H_