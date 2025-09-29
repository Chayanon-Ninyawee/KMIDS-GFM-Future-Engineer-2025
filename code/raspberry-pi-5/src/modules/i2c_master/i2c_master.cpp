#include "i2c_master.h"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <wiringPiI2C.h>

I2cMaster::I2cMaster(uint8_t slave_addr)
    : slaveAddr_(slave_addr)
    , fd_(-1) {
    fd_ = wiringPiI2CSetup(slaveAddr_);
    if (fd_ == -1) {
        printf("Failed to initialize I2C communication with slave 0x%X\n", slaveAddr_);
    } else {
        printf("I2C communication initialized with slave 0x%X\n", slaveAddr_);
    }
}

I2cMaster::~I2cMaster() {
    // nothing to clean up for wiringPiI2C
}

bool I2cMaster::isInitialized() const {
    return fd_ != -1;
}

bool I2cMaster::writeRegister(uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);
    return write(fd_, buffer, sizeof(buffer)) != -1;
}

bool I2cMaster::readRegister(uint8_t reg, uint8_t *data, size_t len) {
    int result = wiringPiI2CReadBlockData(fd_, reg, data, len);
    return (result >= 0);
}

/** Command operations */
bool I2cMaster::sendCommand(uint8_t command) {
    uint8_t cmd_buf[2] = {pico_i2c_mem_addr::COMMAND_ADDR, command};
    return write(fd_, cmd_buf, sizeof(cmd_buf)) != -1;
}

bool I2cMaster::readCommand(uint8_t &outCommand) {
    uint8_t buf = 0;
    if (!readRegister(pico_i2c_mem_addr::COMMAND_ADDR, &buf, 1)) return false;
    outCommand = buf;
    return true;
}

/** Status operations */
bool I2cMaster::readStatus(uint8_t &outStatus) {
    return readRegister(pico_i2c_mem_addr::STATUS_ADDR, &outStatus, 1);
}

bool I2cMaster::getIsRunning(bool &outIsRunning) {
    uint8_t status = 0;
    if (!readStatus(status)) return false;
    outIsRunning = (status & 0x01) != 0;
    return true;
}

bool I2cMaster::getImuReady(bool &outImuReady) {
    uint8_t status = 0;
    if (!readStatus(status)) return false;
    outImuReady = (status & 0x02) != 0;
    return true;
}

/** IMU operations */
bool I2cMaster::readImu(ImuAccel &outAccel, ImuEuler &outEuler) {
    uint8_t buffer[pico_i2c_mem_addr::IMU_DATA_SIZE];
    if (!readRegister(pico_i2c_mem_addr::IMU_DATA_ADDR, buffer, sizeof(buffer))) return false;

    memcpy(&outAccel, buffer, pico_i2c_mem_addr::ACCEL_DATA_SIZE);
    memcpy(&outEuler, buffer + pico_i2c_mem_addr::ACCEL_DATA_SIZE, pico_i2c_mem_addr::EULER_ANGLE_SIZE);
    return true;
}

bool I2cMaster::writeImu(const ImuAccel &accel, const ImuEuler &euler) {
    uint8_t buffer[pico_i2c_mem_addr::IMU_DATA_SIZE];
    memcpy(buffer, &accel, pico_i2c_mem_addr::ACCEL_DATA_SIZE);
    memcpy(buffer + pico_i2c_mem_addr::ACCEL_DATA_SIZE, &euler, pico_i2c_mem_addr::EULER_ANGLE_SIZE);
    return writeRegister(pico_i2c_mem_addr::IMU_DATA_ADDR, buffer, sizeof(buffer));
}

/** Encoder operations */
bool I2cMaster::readEncoder(double &outAngle) {
    uint8_t buffer[pico_i2c_mem_addr::ENCODER_ANGLE_SIZE];
    if (!readRegister(pico_i2c_mem_addr::ENCODER_ANGLE_ADDR, buffer, sizeof(buffer))) return false;

    memcpy(&outAngle, buffer, pico_i2c_mem_addr::ENCODER_ANGLE_SIZE);
    return true;
}

bool I2cMaster::writeEncoder(double angle) {
    uint8_t buffer[pico_i2c_mem_addr::ENCODER_ANGLE_SIZE];
    memcpy(buffer, &angle, pico_i2c_mem_addr::ENCODER_ANGLE_SIZE);
    return writeRegister(pico_i2c_mem_addr::ENCODER_ANGLE_ADDR, buffer, sizeof(buffer));
}

/** Movement operations */
bool I2cMaster::readMovementInfo(double &outMotorSpeed, float &outSteeringPercent) {
    uint8_t buffer[pico_i2c_mem_addr::MOVEMENT_INFO_SIZE];
    if (!readRegister(pico_i2c_mem_addr::MOVEMENT_INFO_ADDR, buffer, sizeof(buffer))) return false;

    memcpy(&outMotorSpeed, buffer, pico_i2c_mem_addr::MOTOR_SPEED_SIZE);
    memcpy(&outSteeringPercent, buffer + pico_i2c_mem_addr::MOTOR_SPEED_SIZE, pico_i2c_mem_addr::STEERING_PERCENT_SIZE);
    return true;
}

bool I2cMaster::writeMovementInfo(double motorSpeed, float steeringPercent) {
    uint8_t buffer[pico_i2c_mem_addr::MOVEMENT_INFO_SIZE];
    memcpy(buffer, &motorSpeed, pico_i2c_mem_addr::MOTOR_SPEED_SIZE);
    memcpy(buffer + pico_i2c_mem_addr::MOTOR_SPEED_SIZE, &steeringPercent, pico_i2c_mem_addr::STEERING_PERCENT_SIZE);
    return writeRegister(pico_i2c_mem_addr::MOVEMENT_INFO_ADDR, buffer, sizeof(buffer));
}
