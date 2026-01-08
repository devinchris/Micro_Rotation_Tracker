#include "../MicroRotationTracker.h"
#include <Arduino_BMI270_BMM150>

Quaternion quat;
EulerAngles angles;
MicroRotationTracker rotation;

float accel[3];
float gyro[3];
float magnet[3];

float accelCalibration[3];
float gyroCalibration[3];

unsigned long lastmillis = 0;
#define SAMPLE_RATE 20              //Hz
#define CALIBRATION_SAMPLES 500 
#define DELAY 20                    //ms

// Magnetometer calibration measured using Magneto 1.2:
static constexpr float MAG_HARD_CAL[3] = {7.0, -35.0, -30.0};
static constexpr float MAG_SOFT_IRON_CAL[3][3] = {
    {1.070, 0.10, 0.01},
    {-0.02, 1.07, 0.04},
    {0.012, 0.04, 1.10}
};

void setup(){
    // === INIT ===
    rotation.init(20.0f);
    Serial.begin(115200);
    
    IMU.begin();

    // MARK: -Calibration
    for(int i = 0; i < CALIBRATION_SAMPLES; i++){
        if(IMU.gyroscopeAvailable() && IMU.accelerationAvailable()){
            IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);
            IMU.readAcceleration(accel[0], accel[1], accel[2]);

            accelCalibration[0] += accel[0];
            accelCalibration[1] += accel[1];
            accelCalibration[2] += (accel[2] - 1.0f); // gravity should result in 1G

            gyroCalibration[0] += gyro[0];
            gyroCalibration[1] += gyro[1];
            gyroCalibration[2] += gyro[2];

            delay(DELAY);
        }
    }

    accelCalibration[0] = accelCalibration[0] / CALIBRATION_SAMPLES;
    accelCalibration[1] = accelCalibration[1] / CALIBRATION_SAMPLES;
    accelCalibration[2] = accelCalibration[2] / CALIBRATION_SAMPLES;

    gyroCalibration[0] = gyroCalibration[0] / CALIBRATION_SAMPLES;
    gyroCalibration[1] = gyroCalibration[1] / CALIBRATION_SAMPLES;
    gyroCalibration[2] = gyroCalibration[2] / CALIBRATION_SAMPLES;

    // Give the filter the calibration data
    rotation.setIMUCalibration(gyroCalibration[0], gyroCalibration[1], gyroCalibration[2],
                               accelCalibration[0], accelCalibration[1], accelCalibration[2]);
    
    rotation.setMagnetometerCorrection(MAG_HARD_CAL, MAG_SOFT_IRON_CAL);

    rotation.setFilterGain(true, 0.01f, 0.1f, 0.4f);
}

void loop(){
    // MARK: -Main Loop
    if(millis() - lastmillis >= (1000.0f / SAMPLE_RATE)){
        getData();

        rotation.calculateRotation(gyro[0], gyro[1], gyro[2],
                                   accel[0], accel[1], accel[2],
                                   magnet[0], -magnet[1], magnet[2]  // Y-axis inverted for BMM150
                                );
        lastmillis = millis();

        quat = rotation.getQuaternion();
        quat = rotation.smoothQuaternion(quat, 0.7f, 0.03f);

        angles = rotation.quaternionToEuler(quat);
        
        // Now go do something with your results!
        // To visualise the Rotation and avoid gimbal lock, use Quaternions instead of Euler Angles!
        doSomething(angles);
    }
}

void getData(){
    if(IMU.gyroscopeAvailable()){
        IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);
    }
    if(IMU.accelerationAvailable()){
        IMU.readAcceleration(accel[0], accel[1], accel[2]);
    }
    if(IMU.magneticFieldAvailable()){
        IMU.readMagneticField(magnet[0], magnet[1], magnet[2]);
    } else {
        magnet[0] = 0.0f; magnet[1] = 0.0f; magnet[2] = 0.0f;
    }
}

void doSomething(EulerAngles _angles){
    Serial.print("Pitch: ");
    Serial.print(angles.pitch);
    Serial.print(", Roll: ");
    Serial.print(angles.roll);
    Serial.print(", Yaw: ");
    Serial.println(angles.yaw);
}
