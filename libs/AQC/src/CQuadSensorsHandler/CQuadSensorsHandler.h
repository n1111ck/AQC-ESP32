#ifndef _CQUADSENSORSHANDLER_H_
#define _CQUADSENSORSHANDLER_H_

#include "CInertialMeasurementUnit/CInertialMeasurementUnit.h"
#include "CBarometer/CBarometer.h"
#include "CCompass/CCompass.h"
#include "DataTypes/DataTypes.h"
#include "CGPS/CGPS.h"
#include "CDistanceDetector/CDistanceDetector.h"

namespace AQC
{
    class CQuadSensorsHandler
    {
        public:
            CQuadSensorsHandler(
                float_t timeSample, 
                uint8_t gpsRxPin, 
                uint8_t gpsTxPin, 
                uint8_t frontTrigPin,
                uint8_t frontEchoPin,
                uint8_t belowTrigPin,
                uint8_t belowEchoPin
            );

            int32_t begin(TwoWire* bus = &Wire);

            FVector3 getAngle();
            FVector3 getAngularVelocity();
            float_t getAltitude();
            float_t getAltitudeVelocity();
            uint32_t getFailed();
            double_t getLongitude();
            double_t getLatitude();
            uint32_t getSatellites();
            void refresh();
            void updateGPS();
            float_t getFrontDistance();
            float_t getBelowDistance();
        private:
            float_t m_timeSample;

            // Attitude sensors
            CInertialMeasurementUnit m_imu;
            CCompass m_compass;
            CBarometer m_baro;

            // Positional sensor
            CGPS m_gps;

            // Collision avoidance sensors
            CDistanceDetector m_front;
            CDistanceDetector m_below;

            void kalman_angle(
                const float_t& dx, 
                const float_t& x_m, 
                float_t& x_s,
                float_t& u, 
                const float_t& sigma_P, 
                const float_t& sigma_O
            );

            void kalman_altitude(
                const float_t& az,
                const float_t& z_m, 
                float_t& vz_s, 
                float_t& z_s, 
                float_t* P,
                const float_t& sigma_P, 
                const float_t& sigma_O
            );

            // Angle
            FVector3 m_angle;
            FVector3 m_angleRate;
            FVector3 m_angleUncertainty;

            // Altitude
            float_t m_altitude;
            float_t m_altitudeVelocity;
            float_t m_altitudeUncertainty[4];
    };
}

#endif //_CISENSOR_H_