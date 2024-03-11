#include "CQuadSensorsHandler.h"

AQC::CQuadSensorsHandler::CQuadSensorsHandler(float_t timeSample):
    m_timeSample(timeSample),
    m_imu({0.18, 0.13, 0.02}),
    m_altitude(),
    m_altitudeVelocity(),
    m_angle({}),
    m_angleRate({}),
    m_angleUncertainty({}),
    m_altitudeUncertainty({})
{
    
}

AQC::CQuadSensorsHandler::~CQuadSensorsHandler()
{
    
}

int32_t
AQC::CQuadSensorsHandler::begin(TwoWire* bus)
{
    int32_t rStatus = 1;

    // Start desired I2C interface
    bus->setClock(400000);
    bus->begin();

    // Try to start IMU
    rStatus += m_imu.begin(bus) > 0 ? 0 : -1;

    // Try to start baro
    rStatus += m_baro.begin(bus) > 0 ? 0 : -2;

    // Try to start baro
    rStatus += m_compass.begin(bus) > 0 ? 0 : -3;

    // All begins succesfull
    if(rStatus == 0)
    {
    }

    return rStatus;
}

void     
AQC::CQuadSensorsHandler::kalman_angle(
    const float_t& dx, 
    const float_t& x_m, 
    float_t& x_s,
    float_t& u, 
    const float_t& sigma_P, 
    const float_t& sigma_O
)
{
    // 1. Predict state
    float_t x_k = x_s + m_timeSample*dx;

    // 2. Predict uncertainty
    float_t u_k = u + pow(m_timeSample, 2)*sigma_P;

    // 3. Kalman gain
    float_t k_k = u_k/(u_k + sigma_O);

    // 4. Update state
    x_s = x_k + k_k*(x_m - x_k);

    // 5. Update uncertainty
    u = (1 - k_k)*u_k;
}

void 
AQC::CQuadSensorsHandler::kalman_altitude(
    const float_t& az,
    const float_t& z_m, 
    float_t& vz_s, 
    float_t& z_s, 
    float_t* P,
    const float_t& sigma_P, 
    const float_t& sigma_O
)
{
    static float_t vz_m = 0.0;
    vz_m += m_timeSample*az;

    // 1. Predict state
    float_t z_k = z_s + vz_s*m_timeSample + 0.5*pow(m_timeSample, 2)*az;
    float_t vz_k = vz_s + m_timeSample*az;

    // 2. Predict uncertainty
    float_t P_k[] = {
        P[0] + P[1]*m_timeSample + P[2]*m_timeSample + P[3]*pow(m_timeSample, 2) + 0.25*pow(m_timeSample, 4)*sigma_P,
        P[1] + P[3]*m_timeSample + 0.5*pow(m_timeSample, 3)*sigma_P,
        P[2] + P[3]*m_timeSample + 0.5*pow(m_timeSample, 3)*sigma_P,
        P[3] + pow(m_timeSample, 2)*sigma_P
    };

    // 3. Kalman gain
    float_t K[] = {
        P_k[0]/(P_k[0] + sigma_O),
        P_k[2]/(P_k[0] + sigma_O)
    };

    // 4. Update state
    z_s = z_k + K[0]*(z_m - z_k);
    vz_s = vz_k + K[1]*(z_m - z_k);

    // 5. Update uncertainty
    P[0] = P_k[0] - K[0]*P_k[0];
    P[1] = - K[0]*P_k[1];
    P[2] = - K[1]*P_k[0];
    P[3] = P_k[3] - K[1]*P_k[1];
}

AQC::FVector3
AQC::CQuadSensorsHandler::getAngle()
{
    return m_angle;
}

AQC::FVector3
AQC::CQuadSensorsHandler::getAngularVelocity()
{
    return m_angleRate;
}

float_t
AQC::CQuadSensorsHandler::getAltitudeVelocity()
{
    return m_altitudeVelocity;
}

float_t
AQC::CQuadSensorsHandler::getAltitude()
{
    return m_altitude;
}


void
AQC::CQuadSensorsHandler::refresh()
{
    FVector3 linAccelIMU;

    // Kalman altitude
    float_t altMeasured, headingMeasured;
        
    // Capture IMU Data
    m_imu.getData(m_angleRate, linAccelIMU);

    // Pitch and Roll
    FVector3 anglesMeasured = m_imu.estimateAnglesFromAccel(linAccelIMU);
    kalman_angle(
        m_angleRate.m_x,
        anglesMeasured.m_x,
        m_angle.m_x,
        m_angleUncertainty.m_x,
        pow(0.04, 2), 
        pow(0.02, 2)
    );
    kalman_angle(
        m_angleRate.m_y,
        anglesMeasured.m_y,
        m_angle.m_y,
        m_angleUncertainty.m_y,
        pow(0.04, 2), 
        pow(0.02, 2)
    );

    // Capture compass data
    headingMeasured = m_compass.getHeading(m_angle);

    // Yaw
    kalman_angle(
        m_angleRate.m_z,
        headingMeasured,
        m_angle.m_z,
        m_angleUncertainty.m_z,
        pow(0.04, 2), 
        pow(0.06, 2)
    );

    // Capture altitude data
    altMeasured = m_baro.getAltitude();

    // Altitude
    kalman_altitude(
        m_imu.getGravity() - m_imu.getInertialZ(linAccelIMU, m_angle),
        altMeasured, 
        m_altitudeVelocity, 
        m_altitude, 
        m_altitudeUncertainty,
        pow(0.1, 2), 
        pow(0.3, 2)
    );
}