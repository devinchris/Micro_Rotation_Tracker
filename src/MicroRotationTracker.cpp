#include "MicroRotationTracker.h"
#include "libs/MadgwickAHRS.h"

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
    /**
     * @brief Set IMU calibration offsets
     * @param gyroBiasX Gyroscope bias in X axis (deg/s)
     * @param gyroBiasY Gyroscope bias in Y axis (deg/s)
     * @param gyroBiasZ Gyroscope bias in Z axis (deg/s)
     * @param accelBiasX Accelerometer bias in X axis (g)
     * @param accelBiasY Accelerometer bias in Y axis (g)
     * @param accelBiasZ Accelerometer bias in Z axis (g)
     */
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
    if(betaNormal <= 0.0f || betaHigh <= 0.0f || betaMax <= 0.0f) {
        // Invalid gain values, ignore
        return;
    }
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
        previousQuat = _quat; // Store previous quaternion for smoothing
        applyMagCalibration(mx, my, mz);
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        previousQuat = _quat; // Store previous quaternion for smoothing
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
Quaternion MicroRotationTracker::smoothQuaternion(Quaternion _quat, float beta, float deadzone) {
    if(deadzone > 0.0f) {
        // DEADZONE calculation
        // Calculate difference between new and previous quaternion
        float dq0 = _quat.q0 - previousQuat.q0;
        float dq1 = _quat.q1 - previousQuat.q1;
        float dq2 = _quat.q2 - previousQuat.q2;
        float dq3 = _quat.q3 - previousQuat.q3;

        // Calculate magnitude of difference
        float diffMagnitude = sqrt(dq0 * dq0 + dq1 * dq1 + dq2 * dq2 + dq3 * dq3);

        // If difference is within deadzone, return previous quaternion
        if(diffMagnitude < deadzone) {
            return previousQuat;
        }
    }

    // NLERP Smoothing Initialization
    return nlerp(_quat, previousQuat, beta);
}

Quaternion MicroRotationTracker::nlerp(const Quaternion& newQ, const Quaternion& prevQ, float alpha) {
    // Calculate NLERP between two quaternions
    // 1. Calculate dot product
    float dot = (newQ.q0 * prevQ.q0) + 
                (newQ.q1 * prevQ.q1) + 
                (newQ.q2 * prevQ.q2) + 
                (newQ.q3 * prevQ.q3);

    // 2. Get shortest path
    float sign = 1.0f;
    if (dot < 0.0f) {
        sign = -1.0f;
    }

    // 3. Linear interpolation
    // Formula: result = prevQ * (1 - alpha) + newQ * alpha * sign
    Quaternion result;
    float invAlpha = 1.0f - alpha;

    result.q0 = (prevQ.q0 * invAlpha) + (newQ.q0 * alpha * sign);
    result.q1 = (prevQ.q1 * invAlpha) + (newQ.q1 * alpha * sign);
    result.q2 = (prevQ.q2 * invAlpha) + (newQ.q2 * alpha * sign);
    result.q3 = (prevQ.q3 * invAlpha) + (newQ.q3 * alpha * sign);

    // 4. Normalize the result quaternion
    float lengthSq = result.q0*result.q0 + result.q1*result.q1 + 
                     result.q2*result.q2 + result.q3*result.q3;

    if (lengthSq > 1e-6f) { // Prevent division by zero
        float invLength = 1.0f / sqrt(lengthSq);
        
        result.q0 *= invLength;
        result.q1 *= invLength;
        result.q2 *= invLength;
        result.q3 *= invLength;
    }

    return result;
}

EulerAngles MicroRotationTracker::quaternionToEuler(const Quaternion& q) {
    EulerAngles angles;
    // Roll (x-axis rotation)
    angles.roll = atan2f(2.0f * (q.q0 * q.q1 + q.q2 * q.q3),
                         1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2)) * QUATERNION_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);
    if (fabs(sinp) >= 1)
        angles.pitch = copysignf(90.0f, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asinf(sinp) * QUATERNION_TO_DEG;

    // Yaw (z-axis rotation)
    angles.yaw = atan2f(2.0f * (q.q0 * q.q3 + q.q1 * q.q2),
                        1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3)) * QUATERNION_TO_DEG;
    return angles;
}