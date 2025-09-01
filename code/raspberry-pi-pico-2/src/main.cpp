#include <cstdint>
#include <cstdio>
#include <hardware/gpio.h>
#include <pico/time.h>

#include "bno085_controller.h"
#include "encoder_controller.h"
#include "i2c_slave.h"
#include "imu_struct.h"
#include "motor_controller.h"
#include "motor_speed_controller.h"
#include "servo_controller.h"

const uint MOSFET_PIN = 19;

const uint I2C0_SDA_PIN = 4;
const uint I2C0_SCL_PIN = 5;

const uint I2C1_SDA_PIN = 2;
const uint I2C1_SCL_PIN = 3;
const uint I2C1_BAUDRATE = 400000;  // 400 kHz
const uint I2C1_SLAVE_ADDR = 0x39;

const uint MOTOR_IN1_PIN = 6;
const uint MOTOR_IN2_PIN = 7;

const uint ENCODER_A_PIN = 10;
const uint ENCODER_B_PIN = 11;
const int ENCODER_PULSE_PER_REV = 28;
const int ENCODER_GEAR_RATIO = 100;

const uint SERVO_PIN = 15;
const float SERVO_MAX_ANGLE = 90.0f;
const uint16_t SERVO_MIN_PULSE_US = 1000;
const uint16_t SERVO_MAX_PULSE_US = 2000;
const uint16_t SERVO_FREQ_HZ = 50;

const float STEERING_MIN_ANGLE = 0.0f;
const float STEERING_MAX_ANGLE = 86.5f;

float toSteeringAngle(float steeringPercent) {
    const float inMin = -100.0f;
    const float inMax = 100.0f;
    const float outMin = STEERING_MIN_ANGLE;
    const float outMax = STEERING_MAX_ANGLE;

    float output = outMin + (steeringPercent - inMin) * (outMax - outMin) / (inMax - inMin);
    return output;
}

int main() {
    gpio_init(MOSFET_PIN);
    gpio_set_dir(MOSFET_PIN, GPIO_OUT);

    gpio_put(MOSFET_PIN, 1);

    stdio_init_all();

    Bno085Controller imu(i2c0, I2C0_SDA_PIN, I2C0_SCL_PIN);

    i2c_slave::contextInit();
    i2c_slave::i2cInit(i2c1, I2C1_SLAVE_ADDR, I2C1_SDA_PIN, I2C1_SCL_PIN, I2C1_BAUDRATE);

    MotorController motor(MOTOR_IN1_PIN, MOTOR_IN2_PIN);
    EncoderController encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_PULSE_PER_REV, ENCODER_GEAR_RATIO);
    MotorSpeedController motorPid(motor, encoder);

    ServoController servo(SERVO_PIN, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US, SERVO_FREQ_HZ);

    motor.begin();
    encoder.begin();
    motorPid.setTargetRPS(0.0);

    servo.begin();
    servo.setAngle(toSteeringAngle(0.0f));

    while (!imu.begin()) {
        sleep_ms(1000);
    }

    imu.setCalibrationConfig(0b0000000);
    sleep_ms(1000);

    i2c_slave::setIsRunning(false);
    i2c_slave::setIsImuReady(true);

    imu.enableRotation(8);
    imu.enableAccelerometer(8);

    ImuAccel accel;
    ImuEuler euler;

    double motorSpeed = 1001.0;
    float steeringPercent = 0.0f;

    uint32_t lastImuUpdate = 0;
    const uint32_t imuInterval = 4;

    uint32_t lastMotorPidUpdate = 0;
    const uint32_t motorPidInterval = 8;
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        i2c_slave::setIsRunning(true);

        if (now - lastImuUpdate >= imuInterval) {
            if (imu.wasReset()) {
                imu.enableRotation(8);
                imu.enableAccelerometer(8);
            }

            if (imu.update(accel, euler)) {
                i2c_slave::setImuData(accel, euler);
                // printf("Euler angles: H: %.2f°, P: %.2f°, R: %.2f°\n", euler.h, euler.p, euler.r);
            }

            double encoderAngleToSend = encoder.getAngle();
            if (encoderAngleToSend == 0.0) encoderAngleToSend = 0.01;
            i2c_slave::setEncoderAngle(encoderAngleToSend);

            i2c_slave::getMovementInfo(motorSpeed, steeringPercent);

            lastImuUpdate = now;
        }

        if (now - lastMotorPidUpdate >= motorPidInterval) {
            // TODO: Make this a proper protocol
            if (motorSpeed >= 1002.0) {
                motorPid.setTargetRPS(0.0);
                motor.stop(true);
            } else if (motorSpeed >= 1001.0) {
                motorPid.setTargetRPS(0.0);
                motor.stop(false);
            } else {
                motorPid.setTargetRPS(motorSpeed);
            }

            motorPid.update();

            servo.setAngle(toSteeringAngle(steeringPercent));

            lastMotorPidUpdate = now;
        }

        // printf("%.2f\n", motorPid.getPower());

        tight_loop_contents();
    }

    while (true)
        tight_loop_contents();
}
