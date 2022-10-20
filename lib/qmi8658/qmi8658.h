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
#include <quat.h>

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
  void getQuaternion(float *q1, float *q2, float *q3, float *q4);
  void getEularAngle(float *roll, float *pitch, float *yaw);
  int16_t getTemperature();

public:
  int16_t ax, ay, az, gx, gy, gz;
  float q[4] {1.0f, 0.0f, 0.0f, 0.0f};
  float pitch, roll, yaw;
  unsigned long now, lastTime = 0;
  
private:
  uint8_t buffer[14];
  CaliData cali;
};

/*----------------------------------------------------------------------------------------------
  QMI8658 UI Sensor Configuration Settings and Output Data
*/
///<Configuration Registers>
// TODO: 目前无法同时使用两个8658，需要修复
// #if SECOND_IMU == IMU_QMI8658
// #define ADDRESS 0X6B  //device address
// #else
// #define ADDRESS 0X6B  //device address
// #endif
#define ADDRESS 0X6B
#define WHO_AM_I 0X00 //Device identifier
#define CTRL1 0x02    //Serial Interface and Sensor Enable
#define CTRL2 0x03    //Accelerometer Settings
#define CTRL3 0x04    //Gyroscope Settings
#define CTRL4 0x05    //Magnetometer Settings
#define CTRL5 0x06    //Sensor Data Processing Settings
#define CTRL6 0x07    //Motion on Demand & Attitude Engine ODR Settings
#define CTRL7 0x08    //Enable Sensors and Configure Data Reads
#define CTRL8 0x09    //Reserved – Special Settings

///<Attitude Engine Registers>
#define AE_REG1 0x57
#define AE_REG2 0x58
///<Sensor Data Output Registers>
#define AccX_L 0x35
#define AccX_H 0x36
#define AccY_L 0x37
#define AccY_H 0x38
#define AccZ_L 0x39
#define AccZ_H 0x3A
#define TEMP_L 0x33

#define GyrX_L 0x3B
#define GyrX_H 0x3C
#define GyrY_L 0x3D
#define GyrY_H 0x3E
#define GyrZ_L 0x3F
#define GyrZ_H 0x40

#define dQW_L 0x49
#define dQW_H 0x4A
#define dQX_L 0x4B
#define dQX_H 0x4C
#define dQY_L 0x4D
#define dQY_H 0x4E
#define dQZ_L 0x4F
#define dQZ_H 0x50

#define dVX_L 0x51
#define dVX_H 0x52
#define dVY_L 0x53
#define dVY_H 0x54
#define dVZ_L 0x55
#define dVZ_H 0x56
int16_t readBytes(unsigned char tmp);
//extern QMI8658 _QMI8658;
#endif
