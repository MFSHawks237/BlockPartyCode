/*!@addtogroup HiTechnic
 * @{
 * @defgroup MRIMMUX Motor MUX
 * Matrix Motor and Servo MUX
 * @{
 */

/*
 * $Id: MRIMMUX-driver.h 98 2012-08-04 09:02:53Z xander $
 */

#ifndef __MRIMMUX_H__
#define __MRIMMUX_H__
/** \file MRIMMUX-driver.h
 * \brief Matrix Motor and Servo MUX driver
 *
 * MRIMMUX-driver.h provides an API for the Matrix Motor and Servo MUX driver.
 *
 * Changelog:
 * - 0.1: Initial release
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where its due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.08 AND HIGHER.
 * \author Xander Soldaat (mightor_at_gmail.com)
 * \date 15 July 2009
 * \version 0.1
 * \example MRIMMUX-test1.c
 */

#pragma systemFile

#ifndef __COMMON_H__
#include "common.h"
#endif

#ifndef __MMUX_H__
#include "MMUX-common.h"
#endif

#define MRIMMUX_I2C_ADDR         0x10  /*!< MRIMMUX I2C device address */

#define MRIMMUX_TARG_ENC         0x00  /*!< Target encoder value */
#define MRIMMUX_POWER            0x04  /*!< Motor power */
#define MRIMMUX_MODE_STAT        0x05  /*!< Mode/status */
#define MRIMMUX_CURR_ENC         0x06  /*!< Current encoder value */
#define MRIMMUX_CMD_OFFSET       0x40  /*!< Command register offset */
#define MRIMMUX_ENTRY_SIZE       0x0A  /*!< Number of registers per motor channel */
#define REG_ADDR(CHAN, REG)     MRIMMUX_CMD_OFFSET + (MRIMMUX_ENTRY_SIZE * CHAN) + REG

// Motor mode commands
#define MRIMMUX_CMD_BRAKE        0x00  /*!< Off - brake command */
#define MRIMMUX_CMD_RUN_CSPEED   0x01  /*!< Run with constant speed command */
#define MRIMMUX_CMD_RUN_CPOWER   0x02  /*!< Run with constant power command */
#define MRIMMUX_CMD_FLOAT        0x03  /*!< Off - coast command */
#define MRIMMUX_CMD_RUN_TO_POS   0x04  /*!< Run to and hold position command */
#define MRIMMUX_CMD_RESET_ENC    0x05  /*!< Reset current encoder command */

// Motor status fields
#define MRIMMUX_STAT_UNAVAIL     0x10  /*!< This motor channel cannot be used status */
#define MRIMMUX_STAT_BATT        0x40  /*!< No battery voltage detected status */
#define MRIMMUX_STAT_BUSY        0x80  /*!< Motor is currently executing a Run to and hold position status*/

#define MRIMMUX_MOTOR1           0x01  /*!< Motor connected to channel 1 */
#define MRIMMUX_MOTOR2           0x02  /*!< Motor connected to channel 2 */
#define MRIMMUX_MOTOR3           0x03  /*!< Motor connected to channel 3 */
#define MRIMMUX_MOTOR4           0x04  /*!< Motor connected to channel 4 */

tByteArray MRIMMUX_I2CRequest;    /*!< Array to hold I2C command data */
tByteArray MRIMMUX_I2CReply;      /*!< Array to hold I2C reply data */


// Function prototypes
void MRIMMUXinit();
ubyte MRIMMUXreadStatus(tSensors link, ubyte channel);
bool MRIMMUXreadStatus(tSensors link, ubyte channel, ubyte statusbit);
bool MRIMMUXsendCommand(tSensors link, ubyte channel, byte command);
bool MRIMMUXMotor(tSensors link, ubyte channel, ubyte power, ubyte command);
bool MRIMMUXMotorStop(tSensors link, ubyte channel, bool brake);
bool MRIMMUXMotorEncoderTarget(tSensors link, ubyte channel, long target);
bool MRIMMUXMotorEncoderReset(tSensors link, ubyte channel);
long MRIMMUXMotorEncoder(tSensors link, ubyte channel);

// Simplified API for MMUX
void MMotor(tMUXmotor muxmotor, byte power);
void MMotorEncoderTarget(tMUXmotor muxmotor, long target);
long MMotorEncoder(tMUXmotor muxmotor);
void MMotorEncoderReset(tMUXmotor muxmotor);
bool MMotorBusy(tMUXmotor muxmotor);
void MMotorSetBrake(tMUXmotor muxmotor);
void MMotorSetFloat(tMUXmotor muxmotor);
void MMotorSetPIDSpeedCtrl(tMUXmotor muxmotor, bool constspeed);


/*
 * Initialise the mmuxData array needed for keeping track of motor settings
 */
void MRIMMUXinit(){
  for (int i = 0; i < 4; i++) {
    memset(mmuxData[i].runToTarget[0], false, 4);
    memset(mmuxData[i].brake[0], true, 4);
    memset(mmuxData[i].pidcontrol[0], true, 4);
    mmuxData[i].initialised = true;
  }
}


/**
 * Read the status and mode of the MMUX
 *
 * The status byte is made up of the following bits:
 *
 * | D7 | D6 | D4 | D3 | D2 | D1 | D1 |
 * - D1:D2 - Used for motor mode commands
 * - D5 - MRIMMUX_STAT_UNAVAIL: This motor channel cannot be used
 * - D6 - MRIMMUX_STAT_BATT: No battery voltage detected status
 * - D7 - MRIMMUX_STAT_BUSY: Motor is currently executing a Run to and hold position
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel we want to check the status of
 * @return the status byte
 */
ubyte MRIMMUXreadStatus(tSensors link, ubyte channel) {
  memset(MRIMMUX_I2CRequest, 0, sizeof(tByteArray));

  MRIMMUX_I2CRequest[0] = 2;               // Message size
  MRIMMUX_I2CRequest[1] = MRIMMUX_I2C_ADDR; // I2C Address
  MRIMMUX_I2CRequest[2] = REG_ADDR(channel, MRIMMUX_MODE_STAT);

  if (!writeI2C(link, MRIMMUX_I2CRequest, 1))
    return -1;

  if (!readI2C(link, MRIMMUX_I2CReply, 1))
    return -1;

  return MRIMMUX_I2CReply[0];
}


/**
 * Check if a status bit is set or not for a specified motor channel.
 *
 * The status byte is made up of the following bits:
 *
 * | D7 | D6 | D4 | D3 | D2 | D1 | D1 |
 * - D1:D2 - Used for motor mode commands
 * - D5 - MRIMMUX_STAT_UNAVAIL: This motor channel cannot be used
 * - D6 - MRIMMUX_STAT_BATT: No battery voltage detected status
 * - D7 - MRIMMUX_STAT_BUSY: Motor is currently executing a Run to and hold position
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel we want to check the status of
 * @param statusbit he bit we wish to check
 * @return true if the statusbit is set, false if it isn't
 */
bool MRIMMUXreadStatus(tSensors link, ubyte channel, ubyte statusbit) {
  if (MRIMMUXreadStatus(link, channel) & statusbit)
    return true;
  else
    return false;
}


/**
 * Send a command to the MMUX.
 *
 * - MRIMMUX_CMD_BRAKE: Off - brake command
 * - MRIMMUX_CMD_RUN_CSPEED: Run with constant speed command
 * - MRIMMUX_CMD_RUN_CPOWER: Run with constant power command
 * - MRIMMUX_CMD_FLOAT: Off - coast command
 * - MRIMMUX_CMD_RUN_TO_POS: Run to and hold position command
 * - MRIMMUX_CMD_RESET_ENC: Reset current encoder command
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel the command should apply to
 * @param command the command to be sent to the motor
 * @return true if no error occured, false if it did
 */
bool MRIMMUXsendCommand(tSensors link, ubyte channel, byte command) {
  memset(MRIMMUX_I2CRequest, 0, sizeof(tByteArray));

  MRIMMUX_I2CRequest[0] = 3;               // Message size
  MRIMMUX_I2CRequest[1] = MRIMMUX_I2C_ADDR; // I2C Address
  MRIMMUX_I2CRequest[2] = REG_ADDR(channel, MRIMMUX_MODE_STAT);
  MRIMMUX_I2CRequest[3] = command;

  return writeI2C(link, MRIMMUX_I2CRequest, 0);
}


/**
 * Run motor with specified speed and command
 *
 * Speed is a number from -25 to +25.  All numbers over 25 are treated as 25.
 * - MRIMMUX_CMD_BRAKE: Off - brake command
 * - MRIMMUX_CMD_RUN_CSPEED: Run with constant speed command
 * - MRIMMUX_CMD_RUN_CPOWER: Run with constant power command
 * - MRIMMUX_CMD_FLOAT: Off - coast command
 * - MRIMMUX_CMD_RUN_TO_POS: Run to and hold position command
 * - MRIMMUX_CMD_RESET_ENC: Reset current encoder command
 *
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel the command should apply to
 * @param power the amount of power to apply to the motor, value between -25 and +25
 * @param command the command to issue to the motor
 * @return true if no error occured, false if it did
 */
bool MRIMMUXMotor(tSensors link, ubyte channel, ubyte power, ubyte command) {
  memset(MRIMMUX_I2CRequest, 0, sizeof(tByteArray));

  MRIMMUX_I2CRequest[0] = 4;               // Message size
  MRIMMUX_I2CRequest[1] = MRIMMUX_I2C_ADDR; // I2C Address
  MRIMMUX_I2CRequest[2] = REG_ADDR(channel, MRIMMUX_POWER);
  MRIMMUX_I2CRequest[3] = power;
  MRIMMUX_I2CRequest[4] = command;

  return writeI2C(link, MRIMMUX_I2CRequest, 0);
}

/**
 * Set target encoder for specified motor channel
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel to set target encoder for
 * @param target the target encoder value
 * @return true if no error occured, false if it did
 */
bool MRIMMUXMotorEncoderTarget(tSensors link, ubyte channel, long target) {
  memset(MRIMMUX_I2CRequest, 0, sizeof(tByteArray));

  MRIMMUX_I2CRequest[0] = 6;               // Message size
  MRIMMUX_I2CRequest[1] = MRIMMUX_I2C_ADDR; // I2C Address
  MRIMMUX_I2CRequest[2] = REG_ADDR(channel, MRIMMUX_TARG_ENC);
  MRIMMUX_I2CRequest[3] = (target >> 24) & 0xFF;
  MRIMMUX_I2CRequest[4] = (target >> 16) & 0xFF;
  MRIMMUX_I2CRequest[5] = (target >>  8) & 0xFF;
  MRIMMUX_I2CRequest[6] = (target >>  0) & 0xFF;

  return writeI2C(link, MRIMMUX_I2CRequest, 0);
}

/**
 * Reset target encoder for specified motor channel, use only at
 * the start of your program.  If you are using the standard NXT wheels
 * you will not run into problems with a wrap-around for the first 500kms
 * or so.
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel the command should apply to
 * @return true if no error occured, false if it did
 */
bool MRIMMUXMotorEncoderReset(tSensors link, ubyte channel) {
  return MRIMMUXsendCommand(link, channel, MRIMMUX_CMD_RESET_ENC);
}


/**
 * Fetch the current encoder value for specified motor channel
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel to set target encoder for
 * @return the current value of the encoder
 */
long MRIMMUXMotorEncoder(tSensors link, ubyte channel) {
  memset(MRIMMUX_I2CRequest, 0, sizeof(tByteArray));

  MRIMMUX_I2CRequest[0] = 2;               // Message size
  MRIMMUX_I2CRequest[1] = MRIMMUX_I2C_ADDR; // I2C Address
  MRIMMUX_I2CRequest[2] = REG_ADDR(channel, MRIMMUX_CURR_ENC);

  writeI2C(link, MRIMMUX_I2CRequest, 4);

  readI2C(link, MRIMMUX_I2CReply, 4);

  return ((int)MRIMMUX_I2CReply[0] << 24) +
         ((int)MRIMMUX_I2CReply[1] << 16) +
         ((int)MRIMMUX_I2CReply[2] <<  8) +
          (int)MRIMMUX_I2CReply[3];
}


/**
 * Stop the specified motor
 *
 * Note: this is part of the advanced MMUX API.
 * @param link the MMUX port number
 * @param channel the channel to set target encoder for
 * @param brake use braking or floating to stop the motor
 * @return the current value of the encoder
 */
bool MRIMMUXMotorStop(tSensors link, ubyte channel, bool brake) {
  if (brake)
    return MRIMMUXsendCommand(link, channel, MRIMMUX_CMD_BRAKE);
  else
    return MRIMMUXsendCommand(link, channel, MRIMMUX_CMD_FLOAT);
}


/**
 * Run motor with specified speed.
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 * @param power power the amount of power to apply to the motor, value between -25 and +25
 */
void MMotor(tMUXmotor muxmotor, byte power) {
  bool brake;
  if (power == 0) {
    brake = mmuxData[SPORT(muxmotor)].brake[MPORT(muxmotor)];
    MRIMMUXMotorStop((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor), brake);
    return;
  }

  if (mmuxData[SPORT(muxmotor)].runToTarget[MPORT(muxmotor)]) {
    MRIMMUXMotor((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor), power, MRIMMUX_CMD_RUN_TO_POS);
    mmuxData[SPORT(muxmotor)].runToTarget[MPORT(muxmotor)] = false;
  } else if (mmuxData[SPORT(muxmotor)].pidcontrol[MPORT(muxmotor)]) {
    MRIMMUXMotor((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor), power, MRIMMUX_CMD_RUN_CSPEED);
  } else {
    MRIMMUXMotor((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor), power, MRIMMUX_CMD_RUN_CPOWER);
  }
}


/**
 * Set encoder target for specified motor channel.
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 * @param target the encoder target value
 */
void MMotorEncoderTarget(tMUXmotor muxmotor, long target) {
  MRIMMUXMotorEncoderTarget((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor), target);
  mmuxData[SPORT(muxmotor)].runToTarget[MPORT(muxmotor)] = true;
}


/**
 * Fetch the current encoder value for specified motor channel
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 * @return the current value of the encoder
 */
long MMotorEncoder(tMUXmotor muxmotor) {
  return MRIMMUXMotorEncoder((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor));
}


/**
 * Reset target encoder for specified motor channel, use only at
 * the start of your program.  If you are using the standard NXT wheels
 * you will not run into problems with a wrap-around for the first 500kms
 * or so.
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 */
void MMotorEncoderReset(tMUXmotor muxmotor) {
  MRIMMUXMotorEncoderReset((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor));
}


/**
 * Check if the specified motor is still busy doing a "run to and hold".
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 * @return true if the motor is still busy doing a "run to and hold", false if it's done
 */
bool MMotorBusy(tMUXmotor muxmotor) {
  return MRIMMUXreadStatus((tSensors)SPORT(muxmotor), (ubyte)MPORT(muxmotor), MRIMMUX_STAT_BUSY);
}


/**
 * Set the stopping method for the specified motor to brake.
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 */
void MMotorSetBrake(tMUXmotor muxmotor) {
  mmuxData[SPORT(muxmotor)].brake[MPORT(muxmotor)] = true;
}


/**
 * Set the stopping method for the specified motor to float.
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 */
void MMotorSetFloat(tMUXmotor muxmotor) {
  mmuxData[SPORT(muxmotor)].brake[MPORT(muxmotor)] = false;
}


/**
 * Set the motor speed control.
 *
 * Note: this is part of the simplified MMUX API.
 * @param muxmotor the MMUX motor
 * @param constspeed if set to true, use constant speed control, if set to false use power control only
 */
void MMotorSetPIDSpeedCtrl(tMUXmotor muxmotor, bool constspeed) {
  mmuxData[SPORT(muxmotor)].pidcontrol[MPORT(muxmotor)] = constspeed;
}

#endif //  __MRIMMUX_H__

/*
 * $Id: MRIMMUX-driver.h 98 2012-08-04 09:02:53Z xander $
 */
/* @} */
/* @} */
