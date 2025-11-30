#include <MadgwickAHRS.h>
#include "MicroRotationTracker.h"

MicroRotationTracker::MicroRotationTracker() {
    // Constructor
}

void MicroRotationTracker::init(float frequency) {
    FREQUENCY = frequency;
    filter.begin(FREQUENCY);
    lastUpdateTime = 0;
}

// === SET CALIBRATION ====
void MicroRotationTracker::setIMUCalibration(float gyroBiasX, float gyroBiasY, float gyroBiasZ,
                                             float accelBiasX, float accelBiasY, float accelBiasZ) {
    gyroBias[0] = gyroBiasX;
    gyroBias[1] = gyroBiasY;
    gyroBias[2] = gyroBiasZ;

    accelBias[0] = accelBiasX;
    accelBias[1] = accelBiasY;
    accelBias[2] = accelBiasZ;
}

void MicroRotationTracker::setMagnetometerCorrection(const float hardIron[3], const float softIron[3][3]){
    for(int i=0; i<3; i++){
        MAG_HARD_IRON_CORRECTION[i] = hardIron[i];
        for(int j=0; j<3; j++){
            MAG_SOFT_IRON_CORRECTION[i][j] = softIron[i][j];
        }
    }
}

void MicroRotationTracker::setFilterGain(bool _useAdaptivegain, float betaNormal, float betaHigh, float betaMax) {
    useAdaptiveGain = _useAdaptivegain;
    if(_useAdaptivegain){
        BETA_NORMAL = betaNormal;
        BETA_HIGH = betaHigh;
        BETA_MAX = betaMax;
    } else {
        filter.beta = betaNormal; // Set to normal gain if not using adaptive
    }
}

// ==== MAIN UPDATE FUNCTION ==== //
void MicroRotationTracker::calculateRotation(float gx, float gy, float gz,
                                             float ax, float ay, float az,
                                             float mx, float my, float mz) 
{
    if(mx != 0.0f || my != 0.0f || mz != 0.0f) useMag = true;
    else useMag = false;

    // Apply calibration biases
    gx -= gyroBias[0];  gy -= gyroBias[1];  gz -= gyroBias[2];
    ax -= accelBias[0]; ay -= accelBias[1]; az -= accelBias[2];

    if(useAdaptiveGain) {
        filter.beta = calcFilterGain(ax, ay, az);
    }

    if(useMag) {
        applyMagCalibration(mx, my, mz);
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        filter.updateIMU(gx, gy, gz, ax, ay, az);
    }

    _quat.q0 = filter.q0;
    _quat.q1 = filter.q1;
    _quat.q2 = filter.q2;
    _quat.q3 = filter.q3;
}

float MicroRotationTracker::calcFilterGain(float ax, float ay, float az) {
    // Calculate the magnitude of the acceleration vector
    float accMagnitude = sqrt(ax * ax + ay * ay + az * az);
    float deviation = fabs(accMagnitude - 1.0f); // Deviation from 1g

    // Map deviation to filter gain
    if(deviation < 0.1f) {
        return BETA_NORMAL; // Stable
    } else if(deviation < 0.5f) {
        return BETA_HIGH; // Moderate movement
    } else {
        return BETA_MAX; // Aggressive movement
    }
}

void MicroRotationTracker::applyMagCalibration(float& mx, float& my, float& mz) {
    // Hard iron correction
    mx -= MAG_HARD_IRON_CORRECTION[0];
    my -= MAG_HARD_IRON_CORRECTION[1];
    mz -= MAG_HARD_IRON_CORRECTION[2];

    // Soft iron correction
    float calibratedMx = MAG_SOFT_IRON_CORRECTION[0][0] * mx +
                         MAG_SOFT_IRON_CORRECTION[0][1] * my +
                         MAG_SOFT_IRON_CORRECTION[0][2] * mz;

    float calibratedMy = MAG_SOFT_IRON_CORRECTION[1][0] * mx +
                         MAG_SOFT_IRON_CORRECTION[1][1] * my +
                         MAG_SOFT_IRON_CORRECTION[1][2] * mz;

    float calibratedMz = MAG_SOFT_IRON_CORRECTION[2][0] * mx +
                         MAG_SOFT_IRON_CORRECTION[2][1] * my +
                         MAG_SOFT_IRON_CORRECTION[2][2] * mz;

    mx = calibratedMx;
    my = calibratedMy;
    mz = calibratedMz;
}

Quaternion MicroRotationTracker::getQuaternion(){
    return _quat;
}

EulerAngles MicroRotationTracker::getEulerAngles(){
    // Madgwick calculates Euler angles internally
    // Beware Euler angles cause gimbal lock
    _euler.roll = filter.getRoll();
    _euler.pitch = filter.getPitch();
    _euler.yaw = filter.getYaw();
    return _euler;
}

// ===========================
// ======== SMOOTHING ========
// ===========================
void MicroRotationTracker::initSmoothing(float beta, float deadzone) {
    // NLERP Smoothing Initialization
    // TODO!
}
