#include "CInertialMeasurementUnit.h"

AQC::CInertialMeasurementUnit::CInertialMeasurementUnit(FVector3 accelOffset):
    m_linAccelOffset(accelOffset),
    m_gravity(0.0)
{
    
}

AQC::CInertialMeasurementUnit::~CInertialMeasurementUnit()
{
    
}

int32_t
AQC::CInertialMeasurementUnit::begin(TwoWire* bus)
{
    // Begin Adafruit Sensor
    int32_t rState = Adafruit_MPU6050::begin(MPU6050_I2CADDR_DEFAULT, bus, 0) ? 0 : SErrorCode::s_IMU; 

    // Begin successfull
    if(rState)
    {
        // Desired configurations
        Adafruit_MPU6050::setGyroRange(MPU6050_RANGE_500_DEG);
        Adafruit_MPU6050::setFilterBandwidth(MPU6050_BAND_21_HZ);
        Adafruit_MPU6050::setAccelerometerRange(MPU6050_RANGE_8_G);

        // Run calibration
        calibrate();
    }

    return rState;
}

void
AQC::CInertialMeasurementUnit::calibrate()
{
    sensors_event_t a, g, t;
    FVector3 angularCalibration = {};
    FVector3 linAccel = {};
    float_t gravity = 0.0;
    uint16_t gravityCount = 0;
    for (uint32_t i = 0; i < s_NUM_OF_CYCLES_CALIBRATE; i++)
    {
        Adafruit_MPU6050::getEvent(&a, &g, &t);

        // Sum angular values
        angularCalibration.m_x -= g.gyro.x;
        angularCalibration.m_y -= g.gyro.y;
        angularCalibration.m_z -= g.gyro.z;
        linAccel.m_x = a.acceleration.x + m_linAccelOffset.m_x;
        linAccel.m_y = a.acceleration.y + m_linAccelOffset.m_y;
        linAccel.m_z = a.acceleration.z + m_linAccelOffset.m_z;

        // Block erroneous accelerometer values
        if(linAccel.norm() < 2*s_COMMON_GRAVITY)
        {
            gravity -= linAccel.norm();
            gravityCount++;
        }
    }
    m_angVelOffset = angularCalibration / static_cast<double_t>(s_NUM_OF_CYCLES_CALIBRATE);
    m_gravity = gravity / static_cast<double_t>(gravityCount);
}

float_t
AQC::CInertialMeasurementUnit::getInertialZ(const FVector3& accel, const FVector3& angle)
{
    return -sin(-angle.m_y)*accel.m_x + 
        (cos(-angle.m_y)*sin(-angle.m_x))*accel.m_y + 
        (cos(-angle.m_y)*cos(-angle.m_x))*accel.m_z;
}

void
AQC::CInertialMeasurementUnit::getData(FVector3& angularRate, FVector3& linAccel)
{
    FVector3 rVector;
    sensors_event_t a, g, t;

    Adafruit_MPU6050::getEvent(&a, &g, &t);

    // Angular Rate
    angularRate.m_x = g.gyro.x + m_angVelOffset.m_x;
    angularRate.m_y = -(g.gyro.y + m_angVelOffset.m_y);
    angularRate.m_z = g.gyro.z + m_angVelOffset.m_z;

    // Linear acceleration
    linAccel.m_x = a.acceleration.x + m_linAccelOffset.m_x;
    linAccel.m_y = -(a.acceleration.y + m_linAccelOffset.m_y);
    linAccel.m_z = a.acceleration.z + m_linAccelOffset.m_z;
}

AQC::FVector3
AQC::CInertialMeasurementUnit::estimateAnglesFromAccel(FVector3 linAccel)
{
    FVector3 rVector = {};

    rVector.m_x = atan2(linAccel.m_y, sqrt(pow(linAccel.m_x, 2) + pow(linAccel.m_z, 2)));
    rVector.m_y = -atan2(linAccel.m_x, sqrt(pow(linAccel.m_y, 2) + pow(linAccel.m_z, 2)));
    rVector.m_z = 0.0;

    return rVector;
}

float_t
AQC::CInertialMeasurementUnit::getGravity()
{
    return m_gravity;
}
