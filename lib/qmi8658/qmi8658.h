/*
 * @Description: QMI8658
 * @Author: ELEGOO
 * @Date: 2019-07-10 16:46:17
 * @LastEditTime: 2021-04-14 14:05:33
 * @LastEditors: Changhua
 */

#ifndef _QMI8658_H_

#define _QMI8658_H_

#define CALI_DATA_NUM	300

#include <Arduino.h>

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
#define ADDRESS 0X6B  //device address
#define WHO_AM_I 0X00 //Device identifier
#define CTRL1 0x02    //Serial Interface and Sensor Enable
#define CTRL2 0x03    //Accelerometer Settings
#define CTRL3 0x04    //Gyroscope Settings
#define CTRL4 0X05    //Magnetometer Settings
#define CTRL5 0X06    //Sensor Data Processing Settings
#define CTRL7 0x08    //Enable Sensors and Configure Data Reads
#define CTRL8 0X09    //Reserved – Special Settings

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
int16_t readBytes(unsigned char tmp);
//extern QMI8658 _QMI8658;
#endif
