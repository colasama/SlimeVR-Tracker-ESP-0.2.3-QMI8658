/*
 * @Description: QMI8658
 * @Author: ELEGOO
 * @Date: 2019-07-10 16:46:17
 * @LastEditTime: 2021-04-23 15:08:40
 * @LastEditors: Changhua
 */
#include "QMI8658.h"
#include "I2Cdev.h"


const uint8_t cmd[] = {AccX_L, AccY_L, AccZ_L, TEMP_L, GyrX_L, GyrY_L, GyrZ_L};
#define QFABS(x)		(((x)<0.0f)?(-1.0f*(x)):(x))
#define ONE_G			(9.807f)
#define TYPICAL_SENSITIVITY_LSB 65.6
#define ENABLE_CALIBRATION false
#define SSVT_A 8192 // for 4G Sensibility
#define SSVT_G 64 // for 512 dps
// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);
static qmi8658_cali g_cali;
static CaliData cali;
/*
  QMI8658 UI Sensor Configuration Settings and Output Data
----------------------------------------------------------------------------------------------*/
int16_t readBytes(unsigned char tmp)
{
  uint8_t buffer[14];
  I2Cdev::readBytes(ADDRESS, tmp, 2, buffer);
  return ((buffer[1] << 8) | buffer[0]);
}

void qmi8658_delay(unsigned int ms)
{
	extern  void delay_ms(u32 ms);

	delay_ms(ms);
}

// about calibration
void onDemandCalibration(void)
{
	Serial.println("QMI8658 Calibration start...");
	I2Cdev::writeByte(ADDRESS, Qmi8658Register_Reset, 0xb0);
	delay(10);	// delay
	I2Cdev::writeByte(ADDRESS, CTRL9, (unsigned char)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
	delay(2200);	// delay 2000ms above
	I2Cdev::writeByte(ADDRESS, CTRL9, (unsigned char)qmi8658_Ctrl9_Cmd_NOP);
	delay(100);	// delay
	Serial.println("QMI8658 Calibration done!");
}

void qmi8658_data_cali_accumulated(unsigned char sensor, float data[3]) {
	if(sensor == 1) { // acc
		if(cali.accCaliNum == 0) {
			cali.accBias[0] = 0.0f;
			cali.accBias[1] = 0.0f;
			cali.accBias[2] = 0.0f;
			cali.lastAcc[0] = data[0];
			cali.lastAcc[1] = data[1];
			cali.lastAcc[2] = data[2];
			cali.accCaliNum++;
		}
		else if(cali.accCaliNum < QMI8658_CALI_DATA_NUM) {
			cali.accBias[0] += data[0] - cali.lastAcc[0];
			cali.accBias[1] += data[1] - cali.lastAcc[1];
			cali.accBias[2] += data[2] - cali.lastAcc[2];
			cali.accCaliNum++;
		} else if(cali.accCaliNum == QMI8658_CALI_DATA_NUM) {
			cali.accBias[0] = cali.accBias[0] / cali.accCaliNum;
			cali.accBias[1] = cali.accBias[1] / cali.accCaliNum;
			cali.accBias[2] = cali.accBias[2] / cali.accCaliNum;
		} else {
			data[0] = data[0] - cali.accBias[0];
			data[1] = data[1] - cali.accBias[1];
			data[2] = data[2] - cali.accBias[2];
		}
	} else {
		if(cali.gyroCaliNum == 0) {
			cali.gyroBias[0] = 0.0f;
			cali.gyroBias[1] = 0.0f;
			cali.gyroBias[2] = 0.0f;
			cali.lastGyro[0] = data[0];
			cali.lastGyro[1] = data[1];
			cali.lastGyro[2] = data[2];
			cali.gyroCaliNum++;
		}
		else if(cali.gyroCaliNum < QMI8658_CALI_DATA_NUM) {
			cali.gyroBias[0] += data[0] - cali.lastGyro[0];
			cali.gyroBias[1] += data[1] - cali.lastGyro[1];
			cali.gyroBias[2] += data[2] - cali.lastGyro[2];
			cali.gyroCaliNum++;
		} else if(cali.gyroCaliNum == QMI8658_CALI_DATA_NUM) {
			cali.gyroBias[0] = cali.gyroBias[0] / cali.gyroCaliNum;
			cali.gyroBias[1] = cali.gyroBias[1] / cali.gyroCaliNum;
			cali.gyroBias[2] = cali.gyroBias[2] / cali.gyroCaliNum;
		} else {
			data[0] = data[0] - cali.gyroBias[0];
			data[1] = data[1] - cali.gyroBias[1];
			data[2] = data[2] - cali.gyroBias[2];
		}
	}
}

void qmi8658_data_cali(unsigned char sensor, float data[3])
{
	float data_diff[3];

	if(sensor == 1) 	// accel
	{
		data_diff[0] = QFABS((data[0]-g_cali.acc_last[0]));
		data_diff[1] = QFABS((data[1]-g_cali.acc_last[1]));
		data_diff[2] = QFABS((data[2]-g_cali.acc_last[2]));
		g_cali.acc_last[0] = data[0];
		g_cali.acc_last[1] = data[1];
		g_cali.acc_last[2] = data[2];

//		qmi8658_log("acc diff : %f	", (data_diff[0]+data_diff[1]+data_diff[2]));
		if((data_diff[0]+data_diff[1]+data_diff[2]) < 0.05f)
		{
			if(g_cali.acc_cali_num == 0)
			{
				g_cali.acc_sum[0] = 0.0f;
				g_cali.acc_sum[1] = 0.0f;
				g_cali.acc_sum[2] = 0.0f;
			}
			if(g_cali.acc_cali_num < QMI8658_CALI_DATA_NUM)
			{
				g_cali.acc_cali_num++;
				g_cali.acc_sum[0] += data[0];
				g_cali.acc_sum[1] += data[1];
				g_cali.acc_sum[2] += data[2];
				if(g_cali.acc_cali_num == QMI8658_CALI_DATA_NUM)
				{
					if((g_cali.acc_cali_flag == 0)&&(data[2]<11.8f)&&(data[2]>7.8f))
					{
						g_cali.acc_sum[0] = g_cali.acc_sum[0]/QMI8658_CALI_DATA_NUM;
						g_cali.acc_sum[1] = g_cali.acc_sum[1]/QMI8658_CALI_DATA_NUM;
						g_cali.acc_sum[2] = g_cali.acc_sum[2]/QMI8658_CALI_DATA_NUM;

						g_cali.acc_bias[0] = 0.0f - g_cali.acc_sum[0];
						g_cali.acc_bias[1] = 0.0f - g_cali.acc_sum[1];
						g_cali.acc_bias[2] = 9.807f - g_cali.acc_sum[2];
						g_cali.acc_cali_flag = 1;
					}
					g_cali.imu_static_flag = 1;
					Serial.println("qmi8658 acc static!!!");
				}
			}

			if(g_cali.imu_static_flag)
			{
				if(g_cali.acc_fix_flag == 0)
				{
					g_cali.acc_fix_flag = 1;
					g_cali.acc_fix[0] = data[0];
					g_cali.acc_fix[1] = data[1];
					g_cali.acc_fix[2] = data[2];
				}
			}
			else
			{
				g_cali.acc_fix_flag = 0;
				g_cali.gyr_fix_flag = 0;
			}
		}
		else
		{
			g_cali.acc_cali_num = 0;
			g_cali.acc_sum[0] = 0.0f;
			g_cali.acc_sum[1] = 0.0f;
			g_cali.acc_sum[2] = 0.0f;

			g_cali.imu_static_flag = 0;
			g_cali.acc_fix_flag = 0;
			g_cali.gyr_fix_flag = 0;
		}

		if(g_cali.acc_fix_flag)
		{
			if(g_cali.acc_fix_index != 0)
				g_cali.acc_fix_index = 0;
			else
				g_cali.acc_fix_index = 1;

			data[0] = g_cali.acc_fix[0] + g_cali.acc_fix_index*0.01f;
			data[1] = g_cali.acc_fix[1] + g_cali.acc_fix_index*0.01f;
			data[2] = g_cali.acc_fix[2] + g_cali.acc_fix_index*0.01f;
		}
		if(g_cali.acc_cali_flag)
		{
			g_cali.acc[0] = data[0] + g_cali.acc_bias[0];
			g_cali.acc[1] = data[1] + g_cali.acc_bias[1];
			g_cali.acc[2] = data[2] + g_cali.acc_bias[2];
			data[0] = g_cali.acc[0];
			data[1] = g_cali.acc[1];
			data[2] = g_cali.acc[2];
		}
		else
		{
			g_cali.acc[0] = data[0];
			g_cali.acc[1] = data[1];
			g_cali.acc[2] = data[2];
		}
	}
	else if(sensor == 2)			// gyroscope
	{
		data_diff[0] = QFABS((data[0]-g_cali.gyr_last[0]));
		data_diff[1] = QFABS((data[1]-g_cali.gyr_last[1]));
		data_diff[2] = QFABS((data[2]-g_cali.gyr_last[2]));
		g_cali.gyr_last[0] = data[0];
		g_cali.gyr_last[1] = data[1];
		g_cali.gyr_last[2] = data[2];
		
//		qmi8658_log("gyr diff : %f	\n", (data_diff[0]+data_diff[1]+data_diff[2]));
		if(((data_diff[0]+data_diff[1]+data_diff[2]) < 0.03f)
			&& ((data[0]>-1.0f)&&(data[0]<1.0f))
			&& ((data[1]>-1.0f)&&(data[1]<1.0f))
			&& ((data[2]>-1.0f)&&(data[2]<1.0f))
			)
		{
			if(g_cali.gyr_cali_num == 0)
			{
				g_cali.gyr_sum[0] = 0.0f;
				g_cali.gyr_sum[1] = 0.0f;
				g_cali.gyr_sum[2] = 0.0f;
			}
			if(g_cali.gyr_cali_num < QMI8658_CALI_DATA_NUM)
			{
				g_cali.gyr_cali_num++;
				g_cali.gyr_sum[0] += data[0];
				g_cali.gyr_sum[1] += data[1];
				g_cali.gyr_sum[2] += data[2];
				if(g_cali.gyr_cali_num == QMI8658_CALI_DATA_NUM)
				{
					if(g_cali.gyr_cali_flag == 0)
					{
						g_cali.gyr_sum[0] = g_cali.gyr_sum[0]/QMI8658_CALI_DATA_NUM;
						g_cali.gyr_sum[1] = g_cali.gyr_sum[1]/QMI8658_CALI_DATA_NUM;
						g_cali.gyr_sum[2] = g_cali.gyr_sum[2]/QMI8658_CALI_DATA_NUM;
			
						g_cali.gyr_bias[0] = 0.0f - g_cali.gyr_sum[0];
						g_cali.gyr_bias[1] = 0.0f - g_cali.gyr_sum[1];
						g_cali.gyr_bias[2] = 0.0f - g_cali.gyr_sum[2];
						g_cali.gyr_cali_flag = 1;
					}
					g_cali.imu_static_flag = 1;
					Serial.println("qmi8658 gyro static!!!");
				}
			}
			
			if(g_cali.imu_static_flag)
			{
				if(g_cali.gyr_fix_flag == 0)
				{
					g_cali.gyr_fix_flag = 1;
					g_cali.gyr_fix[0] = data[0];
					g_cali.gyr_fix[1] = data[1];
					g_cali.gyr_fix[2] = data[2];
				}
			}
			else
			{
				g_cali.gyr_fix_flag = 0;
				g_cali.acc_fix_flag = 0;
			}
		}
		else
		{
			g_cali.gyr_cali_num = 0;
			g_cali.gyr_sum[0] = 0.0f;
			g_cali.gyr_sum[1] = 0.0f;
			g_cali.gyr_sum[2] = 0.0f;
			
			g_cali.imu_static_flag = 0;
			g_cali.gyr_fix_flag = 0;
			g_cali.acc_fix_flag = 0;
		}

		if(g_cali.gyr_fix_flag)
		{		
			if(g_cali.gyr_fix_index != 0)
				g_cali.gyr_fix_index = 0;
			else
				g_cali.gyr_fix_index = 1;

			data[0] = g_cali.gyr_fix[0] + g_cali.gyr_fix_index*0.00005f;
			data[1] = g_cali.gyr_fix[1] + g_cali.gyr_fix_index*0.00005f;
			data[2] = g_cali.gyr_fix[2] + g_cali.gyr_fix_index*0.00005f;
		}

		if(g_cali.gyr_cali_flag)
		{
			g_cali.gyr[0] = data[0] + g_cali.gyr_bias[0];
			g_cali.gyr[1] = data[1] + g_cali.gyr_bias[1];
			g_cali.gyr[2] = data[2] + g_cali.gyr_bias[2];
			data[0] = g_cali.gyr[0];
			data[1] = g_cali.gyr[1];
			data[2] = g_cali.gyr[2];
		}
		else
		{		
			g_cali.gyr[0] = data[0];
			g_cali.gyr[1] = data[1];
			g_cali.gyr[2] = data[2];
		}
	}
}

bool QMI8658::initialize(void)
{
  Wire.begin();
  delay(1000);
  uint8_t chip_id = 0x00;
  do
  {
    I2Cdev::readBytes(ADDRESS, WHO_AM_I, 1, &chip_id);
    Serial.print("WHO_AM_I: 0X");
    Serial.println(chip_id, HEX);
    delay(10);
  } while (chip_id == 0); //确保从机设备在线（强行等待 获取 ID ）
  // 神秘的大概是自校准环节
  onDemandCalibration();
  I2Cdev::writeByte(ADDRESS, CTRL1, 0x40); //Serial Interface and Sensor Enable<串行接口（SPI或I 2 C）地址自动递增>
  I2Cdev::writeByte(ADDRESS, CTRL7, 0x03); //Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer>

  I2Cdev::writeByte(ADDRESS, CTRL2, 0x14); //Accelerometer Settings<0x04: ±2g  500Hz> <0x14: ±4g  500Hz>
  I2Cdev::writeByte(ADDRESS, CTRL3, 0x54); //Gyroscope Settings<0x64 ±2048dps 500Hz> <0x54 ±512dps 500Hz>
  I2Cdev::writeByte(ADDRESS, CTRL5, 0x11); //Sensor Data Processing Settings<Enable Gyroscope Accelerometer 低通滤波>

  delay(2000);

  unsigned short times = 1000; //采样次数
  for (int i = 0; i < times; i++)
  {
    gz = readBytes(GyrX_L);
    gzo += gz;
  }
  gzo /= times; //计算陀螺仪偏移
  return false;
}

// this is scaled data
void QMI8658::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  *ax = readBytes(cmd[0]);
  *ay = readBytes(cmd[1]);
  *az = readBytes(cmd[2]);
  *gx = readBytes(cmd[4]);
  *gy = readBytes(cmd[5]);
  *gz = readBytes(cmd[6]);
    float acc[3];
  acc[0] = readBytes(cmd[0]);
  acc[1] = readBytes(cmd[1]);
  acc[2] = readBytes(cmd[2]);
  qmi8658_data_cali(1, acc);
  *ax = acc[0];
  *ay = acc[1];
  *az = acc[2];
}

void QMI8658::getScaledMotion6(float acc[3], float gyro[3])
{
	unsigned char	buf_reg[12];
	short 			raw_acc_xyz[3];
	short 			raw_gyro_xyz[3];

	raw_acc_xyz[0] = readBytes(cmd[0]);
	raw_acc_xyz[1] = readBytes(cmd[1]);
	raw_acc_xyz[2] = readBytes(cmd[2]);

	raw_gyro_xyz[0] = readBytes(cmd[4]);
	raw_gyro_xyz[1] = readBytes(cmd[5]);
	raw_gyro_xyz[2] = readBytes(cmd[6]);

	acc[0] = (float)(raw_acc_xyz[0]*ONE_G)/SSVT_A;
	acc[1] = (float)(raw_acc_xyz[1]*ONE_G)/SSVT_A;
	acc[2] = (float)(raw_acc_xyz[2]*ONE_G)/SSVT_A;
	gyro[0] = (float)(raw_gyro_xyz[0]*PI)/(SSVT_G*180);		// *pi/180
	gyro[1] = (float)(raw_gyro_xyz[1]*PI)/(SSVT_G*180);
	gyro[2] = (float)(raw_gyro_xyz[2]*PI)/(SSVT_G*180);
  qmi8658_data_cali(1, acc);
  qmi8658_data_cali(2, gyro);
}

void QMI8658::getScaledMotion62(float acc[3], float gyro[3])
{
	unsigned char	buf_reg[12];
	short 			raw_acc_xyz[3];
	short 			raw_gyro_xyz[3];

	raw_acc_xyz[0] = readBytes(cmd[0]);
	raw_acc_xyz[1] = readBytes(cmd[1]);
	raw_acc_xyz[2] = readBytes(cmd[2]);

	raw_gyro_xyz[0] = readBytes(cmd[4]);
	raw_gyro_xyz[1] = readBytes(cmd[5]);
	raw_gyro_xyz[2] = readBytes(cmd[6]);

	acc[0] = (float)(raw_acc_xyz[0]*ONE_G)/SSVT_A;
	acc[1] = (float)(raw_acc_xyz[1]*ONE_G)/SSVT_A;
	acc[2] = (float)(raw_acc_xyz[2]*ONE_G)/SSVT_A;
	gyro[0] = (float)(raw_gyro_xyz[0]*PI)/(SSVT_G*180);		// *pi/180
	gyro[1] = (float)(raw_gyro_xyz[1]*PI)/(SSVT_G*180);
	gyro[2] = (float)(raw_gyro_xyz[2]*PI)/(SSVT_G*180);
  qmi8658_data_cali(1, acc);
  qmi8658_data_cali(2, gyro);
}

void QMI8658::getAcceleration(int16_t* ax, int16_t* ay, int16_t* az) {
  *ax = readBytes(cmd[0]);
  *ay = readBytes(cmd[1]);
  *az = readBytes(cmd[2]);
}

void QMI8658::getRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
  *gx = readBytes(cmd[4]);
  *gy = readBytes(cmd[5]);
  *gz = readBytes(cmd[6]);
}

uint8_t QMI8658::getDeviceID() {
    uint8_t chip_id=0x00;
    I2Cdev::readBytes(ADDRESS, WHO_AM_I, 1, &chip_id);
    return chip_id;
}

// 虚假的testConnection
bool QMI8658::testConnection()
{
    return getDeviceID();
}

// 获取温度，尚未编写
int16_t QMI8658::getTemperature() 
{
  return 0;
}
