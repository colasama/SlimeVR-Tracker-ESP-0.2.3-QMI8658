/*
 * @Description: QMI8658
 * @Author: ELEGOO
 * @Date: 2019-07-10 16:46:17
 * @LastEditTime: 2021-04-14 14:05:33
 * @LastEditors: Changhua
 */

#ifndef _QMI8658_H_

#define _QMI8658_H_
#include <Arduino.h>

#define QMI8658_CALI_DATA_NUM	200

enum Qmi8658Register
{
	Qmi8658Register_WhoAmI = 0,
	Qmi8658Register_Revision,
	Qmi8658Register_Ctrl1,
	Qmi8658Register_Ctrl2,
	Qmi8658Register_Ctrl3,
	Qmi8658Register_Ctrl4,
	Qmi8658Register_Ctrl5,
	Qmi8658Register_Ctrl6,
	Qmi8658Register_Ctrl7,
	Qmi8658Register_Ctrl8,
	Qmi8658Register_Ctrl9,
	Qmi8658Register_Cal1_L = 11,
	Qmi8658Register_Cal1_H,
	Qmi8658Register_Cal2_L,
	Qmi8658Register_Cal2_H,
	Qmi8658Register_Cal3_L,
	Qmi8658Register_Cal3_H,
	Qmi8658Register_Cal4_L,
	Qmi8658Register_Cal4_H,
	Qmi8658Register_FifoWmkTh = 19, 
	Qmi8658Register_FifoCtrl = 20,
	Qmi8658Register_FifoCount = 21,
	Qmi8658Register_FifoStatus = 22,
	Qmi8658Register_FifoData = 23,
	Qmi8658Register_StatusI2CM = 44,
	Qmi8658Register_StatusInt = 45,
	Qmi8658Register_Status0,
	Qmi8658Register_Status1,
	Qmi8658Register_Timestamp_L = 48,
	Qmi8658Register_Timestamp_M,
	Qmi8658Register_Timestamp_H,
	Qmi8658Register_Tempearture_L = 51,
	Qmi8658Register_Tempearture_H,
	Qmi8658Register_Ax_L = 53,
	Qmi8658Register_Ax_H,
	Qmi8658Register_Ay_L,
	Qmi8658Register_Ay_H,
	Qmi8658Register_Az_L,
	Qmi8658Register_Az_H,
	Qmi8658Register_Gx_L = 59,
	Qmi8658Register_Gx_H,
	Qmi8658Register_Gy_L,
	Qmi8658Register_Gy_H,
	Qmi8658Register_Gz_L,
	Qmi8658Register_Gz_H,
	Qmi8658Register_Mx_L = 65,
	Qmi8658Register_Mx_H,
	Qmi8658Register_My_L,
	Qmi8658Register_My_H,
	Qmi8658Register_Mz_L,
	Qmi8658Register_Mz_H,
	Qmi8658Register_firmware_id = 73,
	Qmi8658Register_uuid = 81,

	Qmi8658Register_Pedo_L = 90,
	Qmi8658Register_Pedo_M = 91,
	Qmi8658Register_Pedo_H = 92,
		
	Qmi8658Register_Reset = 96
};

enum qmi8658_Ctrl9Command
{
	qmi8658_Ctrl9_Cmd_NOP					= 0X00,
	qmi8658_Ctrl9_Cmd_GyroBias				= 0X01,
	qmi8658_Ctrl9_Cmd_Rqst_Sdi_Mod			= 0X03,
	qmi8658_Ctrl9_Cmd_Rst_Fifo				= 0X04,
	qmi8658_Ctrl9_Cmd_Req_Fifo				= 0X05,
	qmi8658_Ctrl9_Cmd_I2CM_Write			= 0X06,
	qmi8658_Ctrl9_Cmd_WoM_Setting			= 0x08,
	qmi8658_Ctrl9_Cmd_AccelHostDeltaOffset	= 0x09,
	qmi8658_Ctrl9_Cmd_GyroHostDeltaOffset	= 0x0A,
	qmi8658_Ctrl9_Cmd_EnableExtReset		= 0x0B,
	qmi8658_Ctrl9_Cmd_EnableTap				= 0x0C,	
	qmi8658_Ctrl9_Cmd_EnablePedometer		= 0x0D,
	qmi8658_Ctrl9_Cmd_Motion				= 0x0E,
	qmi8658_Ctrl9_Cmd_CopyUsid				= 0x10,
	qmi8658_Ctrl9_Cmd_SetRpu				= 0x11,
	qmi8658_Ctrl9_Cmd_On_Demand_Cali		= 0xA2,
	qmi8658_Ctrl9_Cmd_Dbg_WoM_Data_Enable	= 0xF8
};

typedef struct qmi8658_cali
{
	float			acc_last[3];
	float			acc[3];
	float			acc_fix[3];
	float			acc_bias[3];
	float			acc_sum[3];

	float			gyr_last[3];
	float			gyr[3];
	float			gyr_fix[3];
	float			gyr_bias[3];    
	float			gyr_sum[3];

	unsigned char	imu_static_flag;
	unsigned char	acc_fix_flag;	
	unsigned char	gyr_fix_flag;
	char			acc_fix_index;
	unsigned char	gyr_fix_index;

	unsigned char	acc_cali_flag;
	unsigned char	gyr_cali_flag;
	unsigned short	acc_cali_num;
	unsigned short	gyr_cali_num;
//    unsigned char	acc_avg_num;
//    unsigned char	gyr_avg_num;
} qmi8658_cali;

typedef struct CaliData {
	float accBias[3];
	float gyroBias[3];
	float lastAcc[3];
	float lastGyro[3];
	int accCaliNum;
	int gyroCaliNum;
} CaliData;

class QMI8658
{
public:
  uint8_t getDeviceID(void);
  bool testConnection(void);
  bool initialize(void);
  void getScaledMotion6(float acc[3], float gyro[3]);
  void getScaledMotion62(float acc[3], float gyro[3]);
  void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
  void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
  void getRotation(int16_t* gx, int16_t* gy, int16_t* gz);
  int16_t getTemperature();

public:
  int16_t ax, ay, az, gx, gy, gz;
  float pith, roll, yaw;
  unsigned long now, lastTime = 0;
  float dt;      //微分时间
  float agz = 0; //角度变量
  long gzo = 0;  //陀螺仪偏移量
  
private:
  uint8_t buffer[14];
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
#define CTRL9 0X10    //

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
void qmi8658_data_cali(unsigned char sensor, float data[3]);
//extern QMI8658 _QMI8658;
#endif
