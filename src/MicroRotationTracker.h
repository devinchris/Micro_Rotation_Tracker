#ifndef MICRO_ROTATION_TRACKER_H
#define MICRO_ROTATION_TRACKER_H

#include "libs/MadgwickAHRS.h"

// Data structs for output data
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

class MicroRotationTracker {
public:
    MicroRotationTracker();

    // ==== INIT ==== //
    // Initialize filter at the start
    void init(float frequency = 100.0f);

    Quaternion smoothQuaternion(Quaternion _quat, float beta=0.7f, float deadzone=0.0f);

    // ==== CALIBRATION ==== //
    // Calibration routine for IMU sensors
    void setIMUCalibration(float gyroBiasX, float gyroBiasY, float gyroBiasZ,
                           float accelBiasX, float accelBiasY, float accelBiasZ);

    void setMagnetometerCorrection(const float hardIron[3], const float softIron[3][3]);

    // ==== Adaptive Filter Gain Adjustment ==== //
    void setFilterGain(bool _useAdaptivegain, float betaNormal=0.01f, float betaHigh=0.1f, float betaMax=0.4f);

    // ==== UPDATE & GETTERS ==== //
    // Update rotation based on sensor data
    void calculateRotation(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx=0.0f, float my=0.0f, float mz=0.0f);

    // Primary getter for Quaternion data
    Quaternion getQuaternion();

    // Secondary getter for Euler Angles
    // Beware of gimbal lock when using Euler angles
    EulerAngles getEulerAngles();

    // Calculate Euler Angles via Quaternion
    // use this if you want to smooth Euler angles directly
    // Gimbal lock may still occur
    EulerAngles quaternionToEuler(const Quaternion& q);

private:
    // Filter variables
    Madgwick filter;
    unsigned long lastUpdateTime;

    // Frequency in Hz
    float FREQUENCY = 100.0f; // Default frequency

    // Magnetometer calibration data
    // Once using the magnetometer you should always calibrate for best results
    // I recommend free software like MagCal or Magneto 1.2
    float MAG_HARD_IRON_CORRECTION[3] = {0.0f, 0.0f, 0.0f};   // x, y, z offsets
    float MAG_SOFT_IRON_CORRECTION[3][3] = {                  // Identity matrix by default
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    float gyroBias[3] = {0.0f, 0.0f, 0.0f};
    float accelBias[3] = {0.0f, 0.0f, 0.0f};

    bool useMag = false;
    bool useAdaptiveGain = false;
    float calcFilterGain(float ax, float ay, float az);

    // adaptive filter gain
    float BETA_NORMAL = 0.01f;
    float BETA_HIGH = 0.1f;
    float BETA_MAX = 0.4f;

    Quaternion previousQuat = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion
    Quaternion nlerp(const Quaternion& newQ, const Quaternion& prevQ, float alpha);

    static constexpr float QUATERNION_TO_DEG = 57.29578f; // 180 / PI

    void applyMagCalibration(float& mx, float& my, float& mz);

    Quaternion _quat;
    EulerAngles _euler;
};

#endif // MICRO_ROTATION_TRACKER_H