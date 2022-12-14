/*
 * @Description: QMI8658
 * @Author: ELEGOO
 * @Date: 2019-07-10 16:46:17
 * @LastEditTime: 2021-04-14 14:05:33
 * @LastEditors: Changhua
 */

#ifndef _QMI8658_H_

#define _QMI8658_H_

#define CALI_DATA_NUM	1000

#include <Arduino.h>
#include "../../src/defines.h"

typedef struct CaliData {
	bool isCalibrated;
	float accOffset[3];
	float gyroOffset[3];
	float accDataSum[3];
	float gyroDataSum[3];
	int caliNum;
} CaliData;

class QMI8658
{
public:
  uint8_t getDeviceID(void);
  bool testConnection(void);
  bool initialize(void);
  void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
  void getScaledMotion6(float acc[3], float gyro[3]);
  void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
  void getRotation(int16_t* gx, int16_t* gy, int16_t* gz);
  void getCalibratedData(float acc[3], float gyro[3]);
  int16_t getTemperature();

public:
  int16_t ax, ay, az, gx, gy, gz;
  float pith, roll, yaw;
  unsigned long now, lastTime = 0;
  
private:
  uint8_t buffer[14];
  CaliData cali;
};

/*----------------------------------------------------------------------------------------------
  QMI8658 UI Sensor Configuration Settings and Output Data
*/
///<Configuration Registers>
#if SECOND_IMU == IMU_QMI8658
#define ADDRESS 0X6A  //device address
#else
#define ADDRESS 0X6B  //device address
#endif
//General Purpose Registers
#define WHO_AM_I 0x00 //Device identifier 设备标识符
#define REVISION_ID 0x01 //Device Revision ID 设备修订ID

//Setup and Control Registers 设置和控制寄存器
#define CTRL1 0x02    //Serial Interface and Sensor Enable
#define CTRL2 0x03    //Accelerometer Settings
#define CTRL3 0x04    //Gyroscope Settings
#define CTRL4 0X05    //Magnetometer Settings
#define CTRL5 0X06    //Sensor Data Processing Settings
#define CTRL7 0x08    //Enable Sensors and Configure Data Reads
#define CTRL8 0X09    //Reserved – Special Settings

//Host Controlled Calibration Registers 主机控制校准寄存器
#define CAL1_L 0x0B //CAL1_L –lower 8 bits.
#define CAL1_H 0x0C //CAL1_H – upper 8 bits.
#define CAL2_L 0x0D //CAL2_L – lower 8 bits. 
#define CAL2_H 0X0E //CAL2_H – upper 8 bits.
#define CAL3_L 0X0F //CAL3_L – lower 8 bits. 
#define CAL3_H 0X10 //CAL3_H – upper 8 bits.
#define CAL4_L 0X11 //CAL4_L – lower 8 bits. 
#define CAL4_H 0X12 //CAL4_H – upper 8 bits.

//FIFO Registers
#define FIFO_WTM_TH 0x19//FIFO watermark level, in ODRs
#define FIFO_CTRL 0x14 //FIFO Setup
#define FIFO_SMPL_CNT 0x15 //FIFO sample count LSBs
#define FIFO_STATUS  0x16// FIFO Status
#define FIFO_DATA  0x17// FIFO Data

//Status Registers
#define I2CM_STATUS 0x2C// I2C Master Status.
#define STATUSINT 0x2D// Sensor Data Availability with the Locking mechanism.
#define STATUS0 0x2E// Output Data Over Run and Data Availability.
#define STATUS1 0x2F //Miscellaneous Status: Wake on Motion, CmdDone(CTRL9 protocol bit).

//Timestamp Register
#define TIMESTAMP_LOWr 0x30//lower 8 bits.
#define TIMESTAMP_MIDr 0x31//middle 8 bits.
#define TIMESTAMP_HIGHr 0x32//upper 8 bits

//Data Output Registers 数据输出寄存器
#define TEMP_L 0x33//Temperature Output Data
#define TEMP_H 0X34

#define AccX_L 0x35//Acceleration
#define AccX_H 0x36
#define AccY_L 0x37
#define AccY_H 0x38
#define AccZ_L 0x39
#define AccZ_H 0x3A

#define GyrX_L 0x3B// Angular Rate
#define GyrX_H 0x3C
#define GyrY_L 0x3D
#define GyrY_H 0x3E
#define GyrZ_L 0x3F
#define GyrZ_H 0x40

#define MX_L 0x41//Magnetic Field
#define MX_H 0x42
#define MY_L 0x43
#define MY_H 0x44
#define MZ_L 0x45
#define MZ_H 0x46

#define dQW_L 0x49 //Quaternion Increment dQW
#define dQW_H 0x4A
#define dQX_L 0x4B
#define dQX_H 0x4C
#define dQY_L 0x4D
#define dQY_H 0x4E
#define dQZ_L 0x4F
#define dQZ_H 0x50

#define dVX_L 0x51//Velocity Increment
#define dVX_H 0x52
#define dVY_L 0x53
#define dVY_H 0x54
#define dVZ_L 0x55
#define dVZ_H 0x56

#define AE_REG1 57 //AttitudeEngine Register 1
#define AE_REG2 58
 
//Reset Register
#defien RESET 0x60 //Soft Reset Register

int16_t readBytes(unsigned char tmp);
//extern QMI8658 _QMI8658;
#endif
