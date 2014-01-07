/*!@addtogroup mindsensors
 * @{
 * @defgroup MSIMU AbsoluteIMU Sensor
 * Mindsensors AbsoluteIMU Sensor (MSIMU) driver
 * @{
 */

/*
 * $Id: mindsensors-imu.h 109 2012-09-25 17:10:26Z xander $
 */

#ifndef __MSIMU_H__
#define __MSIMU_H__
/** \file mindsensors-imu.h
 * \brief Mindsensors AbsoluteIMU Sensor driver
 *
 * mindsensors-imu.h provides an API for the Mindsensors AbsoluteIMU Sensor.
 *
 * Changelog:
 * - 0.1: Initial release
 *
 * Credits:
 * - Big thanks to Mindsensors for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where its due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.51 AND HIGHER.
 * \author Xander Soldaat (mightor_at_gmail.com)
 * \date 23 August 2012
 * \version 0.1
 * \example mindsensors-imu-test1.c
 * \example mindsensors-imu-test2.c
 */

#pragma systemFile

#ifndef __COMMON_H__
#include "common.h"
#endif

#define MSIMU_IMU_I2C_ADDR          0x22  /*!< IMU I2C address */

#define MSIMU_REG_CMD               0x41  /*!< Command register */

#define MSIMU_REG_TILT_ALL_AXES     0x42  /*!< All Axes for Tilt */
#define MSIMU_REG_TILT_X_AXIS       0x42  /*!< X Axis for Tilt */
#define MSIMU_REG_TILT_Y_AXIS       0x43  /*!< Y Axis for Tilt */
#define MSIMU_REG_TILT_Z_AXIS       0x44  /*!< Z Axis for Tilt */

#define MSIMU_REG_ACC_ALL_AXES      0x45  /*!< All Axes for Accel */
#define MSIMU_REG_ACC_X_AXIS        0x45  /*!< X Axis for Accel */
#define MSIMU_REG_ACC_Y_AXIS        0x47  /*!< Y Axis for Accel */
#define MSIMU_REG_ACC_Z_AXIS        0x49  /*!< Z Axis for Accel */

#define MSIMU_REG_COMPASS_HEADING   0x4B  /*!< Compass heading */

#define MSIMU_REG_GYRO_ALL_AXES     0x53  /*!< All Axes for Gyro */
#define MSIMU_REG_GYRO_X_AXIS       0x53  /*!< X Axis for Gyro */
#define MSIMU_REG_GYRO_Y_AXIS       0x55  /*!< Y Axis for Gyro */
#define MSIMU_REG_GYRO_Z_AXIS       0x57  /*!< Z Axis for Gyro */

#define MSIMU_CMD_COMPASS_START_CAL 0x04  /*!< Accelerometer 2G range */
#define MSIMU_CMD_COMPASS_STOP_CAL  0x04  /*!< Accelerometer 2G range */

#define MSIMU_CMD_ACC_RANGE_2G      0x04  /*!< Accelerometer 2G range */
#define MSIMU_CMD_ACC_RANGE_4G      0x08  /*!< Accelerometer 4G range */
#define MSIMU_CMD_ACC_RANGE_8G      0x00  /*!< Accelerometer 8G range */
#define MSIMU_CMD_ACC_RANGE_16G     0x00  /*!< Accelerometer 16G range */

#define MSIMU_TILT_X_AXIS           MSIMU_REG_TILT_X_AXIS
#define MSIMU_TILT_Y_AXIS           MSIMU_REG_TILT_Y_AXIS
#define MSIMU_TILT_Z_AXIS           MSIMU_REG_TILT_Z_AXIS

#define MSIMU_ACC_X_AXIS            MSIMU_REG_ACC_X_AXIS
#define MSIMU_ACC_Y_AXIS            MSIMU_REG_ACC_Y_AXIS
#define MSIMU_ACC_Z_AXIS            MSIMU_REG_ACC_Z_AXIS

#define MSIMU_GYRO_X_AXIS           MSIMU_REG_GYRO_X_AXIS
#define MSIMU_GYRO_Y_AXIS           MSIMU_REG_GYRO_Y_AXIS
#define MSIMU_GYRO_Z_AXIS           MSIMU_REG_GYRO_Z_AXIS


#define MSIMUstartCal(X)            _MSIMUsendCmd(X, MSIMU_CMD_COMPASS_START_CAL)   /*!< Start compass calibration */
#define MSIMUstopCal(X)             _MSIMUsendCmd(X, MSIMU_CMD_COMPASS_STOP_CAL)    /*!< Stop compass calibration */
#define MSIMUsetAccelRange2G(X)     _MSIMUsendCmd(X, MSIMU_CMD_ACC_RANGE_2G)        /*!< Configure accelerometer for 2G range */
#define MSIMUsetAccelRange4G(X)     _MSIMUsendCmd(X, MSIMU_CMD_ACC_RANGE_4G)        /*!< Configure accelerometer for 4G range */
#define MSIMUsetAccelRange8G(X)     _MSIMUsendCmd(X, MSIMU_CMD_ACC_RANGE_8G)        /*!< Configure accelerometer for 8G range */
#define MSIMUsetAccelRange16G(X)    _MSIMUsendCmd(X, MSIMU_CMD_ACC_RANGE_16G)       /*!< Configure accelerometer for 16G range */

#define MSIMUreadTiltXAxis(X)       MSIMUreadTiltAxis(X, MSIMU_REG_TILT_X_AXIS)     /*!< Read Tilt X axis */
#define MSIMUreadTiltYAxis(X)       MSIMUreadTiltAxis(X, MSIMU_REG_TILT_Y_AXIS)     /*!< Read Tilt Y axis */
#define MSIMUreadTiltZAxis(X)       MSIMUreadTiltAxis(X, MSIMU_REG_TILT_Z_AXIS)     /*!< Read Tilt Z axis */

#define MSIMUreadGyroXAxis(X)       MSIMUreadGyroAxis(X, MSIMU_REG_GYRO_X_AXIS)     /*!< Read Gyro X axis */
#define MSIMUreadGyroYAxis(X)       MSIMUreadGyroAxis(X, MSIMU_REG_GYRO_Y_AXIS)     /*!< Read Gyro Y axis */
#define MSIMUreadGyroZAxis(X)       MSIMUreadGyroAxis(X, MSIMU_REG_GYRO_Z_AXIS)     /*!< Read Gyro Z axis */

#define MSIMUreadAccelXAxis(X)      MSIMUreadAccelAxis(X, MSIMU_REG_ACC_X_AXIS)     /*!< Read accelerometer X axis */
#define MSIMUreadAccelYAxis(X)      MSIMUreadAccelAxis(X, MSIMU_REG_ACC_Y_AXIS)     /*!< Read accelerometer Y axis */
#define MSIMUreadAccelZAxis(X)      MSIMUreadAccelAxis(X, MSIMU_REG_ACC_Z_AXIS)     /*!< Read accelerometer Z axis */

tByteArray MSIMU_I2CRequest;    /*!< Array to hold I2C command data */
tByteArray MSIMU_I2CReply;      /*!< Array to hold I2C reply data */

bool _MSIMUsendCMD(tSensors link, ubyte cmd);
int MSIMUreadTiltAxis(tSensors link, ubyte axis);
bool MSIMUreadTiltAxes(tSensors link, int &_x, int &_y, int &_z);
int MSIMUreadGyroAxis(tSensors link, ubyte axis);
bool MSIMUreadGyroAxes(tSensors link, int &_x, int &_y, int &_z);
int MSIMUreadAccelAxis(tSensors link, ubyte axis);
bool MSIMUreadAccelAxes(tSensors link, int &_x, int &_y, int &_z);
int MSIMUreadHeading(tSensors link);


/**
 * Send a command to the sensor
 *
 * Note: this is an internal function and should be called directly
 * @param link the port number
 * @param cmd the command to be sent
 * @return true if no error occured, false if it did
 */
bool _MSIMUsendCMD(tSensors link, ubyte cmd)
{
	MSIMU_I2CRequest[0] = 3;                        // Message size
  MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR;      // I2C Address
	MSIMU_I2CRequest[2] = MSIMU_REG_CMD;  // Register address
	MSIMU_I2CRequest[3] = cmd;  // command

  return writeI2C(link, MSIMU_I2CRequest);
}


/**
 * Retrieve the tilt axis data
 * @param link the port number
 * @param axis the specified axis
 * @return the axis data in degrees per second
 */
int MSIMUreadTiltAxis(tSensors link, ubyte axis){
	MSIMU_I2CRequest[0] = 2;                   // Message size
  MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR;  // I2C Address
	MSIMU_I2CRequest[2] = axis;            // Register address

  if (!writeI2C(link, MSIMU_I2CRequest, MSIMU_I2CReply, 1))
    return 0;

  return (MSIMU_I2CReply[0] >= 128) ? (int)MSIMU_I2CReply[0] - 256 : (int)MSIMU_I2CReply[0];
}


/**
 * Read all three tilt axes
 * @param link the port number
 * @param _x data for x axis in degrees per second
 * @param _y data for y axis in degrees per second
 * @param _z data for z axis in degrees per second
 * @return true if no error occured, false if it did
 */
bool MSIMUreadTiltAxes(tSensors link, int &_x, int &_y, int &_z){
	MSIMU_I2CRequest[0] = 2;                        // Message size
  MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR;      // I2C Address
	MSIMU_I2CRequest[2] = MSIMU_REG_TILT_ALL_AXES;  // Register address

  if (!writeI2C(link, MSIMU_I2CRequest, MSIMU_I2CReply, 3))
    return false;

  _x = (MSIMU_I2CReply[0] >= 128) ? (int)MSIMU_I2CReply[0] - 256 : (int)MSIMU_I2CReply[0];
  _y = (MSIMU_I2CReply[1] >= 128) ? (int)MSIMU_I2CReply[1] - 256 : (int)MSIMU_I2CReply[1];
  _z = (MSIMU_I2CReply[2] >= 128) ? (int)MSIMU_I2CReply[2] - 256 : (int)MSIMU_I2CReply[2];

  return true;
}


/**
 * Read the specified gyro axis
 * @param link the port number
 * @param axis the specified axis
 * @return the axis data in degrees per second
 */
int MSIMUreadGyroAxis(tSensors link, ubyte axis){
	MSIMU_I2CRequest[0] = 2;                   // Message size
  MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR;  // I2C Address
	MSIMU_I2CRequest[2] = axis;            // Register address

  if (!writeI2C(link, MSIMU_I2CRequest, MSIMU_I2CReply, 2))
    return 0;

  return (MSIMU_I2CReply[0] + ((int)(MSIMU_I2CReply[1]<<8)));
}


/**
 * Read all three axes of the gyro
 * @param link the port number
 * @param _x data for x axis in degrees per second
 * @param _y data for y axis in degrees per second
 * @param _z data for z axis in degrees per second
 * @return true if no error occured, false if it did
 */
bool MSIMUreadGyroAxes(tSensors link, int &_x, int &_y, int &_z){
	MSIMU_I2CRequest[0] = 2;                        // Message size
  MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR;      // I2C Address
	MSIMU_I2CRequest[2] = MSIMU_REG_GYRO_ALL_AXES;  // Register address

  if (!writeI2C(link, MSIMU_I2CRequest, MSIMU_I2CReply, 6))
    return false;

  _x = MSIMU_I2CReply[0] + ((int)(MSIMU_I2CReply[1]<<8));
  _y = MSIMU_I2CReply[2] + ((int)(MSIMU_I2CReply[3]<<8));
  _z = MSIMU_I2CReply[4] + ((int)(MSIMU_I2CReply[5]<<8));
  return true;
}


/**
 * Read the specified accelerometer axis
 * @param link the port number
 * @param axis the specific axis
 * @return gravity in mG
 */
int MSIMUreadAccelAxis(tSensors link, ubyte axis){
	MSIMU_I2CRequest[0] = 2;                  // Sending address, register.
	MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR; // I2C Address of accl.
  MSIMU_I2CRequest[2] = axis;

	if (!writeI2C(link, MSIMU_I2CRequest[0], MSIMU_I2CReply, 2))
	  return 0;

	return MSIMU_I2CReply[0] + ((int)(MSIMU_I2CReply[1]<<8));
}


/**
 * Read all of the accelerometer axes
 * @param link the port number
 * @param _x variable to hold X axis data
 * @param _y variable to hold Y axis data
 * @param _z variable to hold Z axis data
 * @return true if no error occured, false if it did
 */
bool MSIMUreadAccelAxes(tSensors link, int &_x, int &_y, int &_z){
	MSIMU_I2CRequest[0] = 2;                        // Message size
  MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR;      // I2C Address
	MSIMU_I2CRequest[2] = MSIMU_REG_ACC_ALL_AXES;  // Register address

  if (!writeI2C(link, MSIMU_I2CRequest, MSIMU_I2CReply, 6))
    return false;

  _x = MSIMU_I2CReply[0] + ((int)(MSIMU_I2CReply[1]<<8));
  _y = MSIMU_I2CReply[2] + ((int)(MSIMU_I2CReply[3]<<8));
  _z = MSIMU_I2CReply[4] + ((int)(MSIMU_I2CReply[5]<<8));
  return true;
}


/**
 * Read the current magnetic compass heading
 * @param link the port number
 * @return the current heading
 */
int MSIMUreadHeading(tSensors link)
{
	MSIMU_I2CRequest[0] = 2;                        // Message size
  MSIMU_I2CRequest[1] = MSIMU_IMU_I2C_ADDR;      // I2C Address
	MSIMU_I2CRequest[2] = MSIMU_REG_COMPASS_HEADING;  // Register address

  if (!writeI2C(link, MSIMU_I2CRequest, MSIMU_I2CReply, 2))
    return 0;

  return MSIMU_I2CReply[0] + ((int)(MSIMU_I2CReply[1]<<8));
}


#endif // __MSIMU_H__

/*
 * $Id: mindsensors-imu.h 109 2012-09-25 17:10:26Z xander $
 */
/* @} */
/* @} */
