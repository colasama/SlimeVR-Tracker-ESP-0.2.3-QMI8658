/*
 * @Description: QMI8658
 * @Author: ELEGOO
 * @Date: 2019-07-10 16:46:17
 * @LastEditTime: 2022-10-20 11:39:22
 * @LastEditors: Colanns
 */
#include "QMI8658.h"
#include "I2Cdev.h"

const uint8_t cmd[] = {AccX_L, AccY_L, AccZ_L, TEMP_L, GyrX_L, GyrY_L, GyrZ_L};
#define QFABS(x) (((x) < 0.0f) ? (-1.0f * (x)) : (x))
#define ONE_G (9.807f)
#define TYPICAL_SENSITIVITY_LSB 65.6
#define ENABLE_CALIBRATION false
#define SSVT_A 8192 // for 4G Sensibility
#define SSVT_G 64	// for 512 dps
// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

/*
  QMI8658 UI Sensor Configuration Settings and Output Data
----------------------------------------------------------------------------------------------*/
int16_t readBytes(unsigned char tmp)
{
	uint8_t buffer[14];
	I2Cdev::readBytes(ADDRESS, tmp, 2, buffer);
	return ((buffer[1] << 8) | buffer[0]);
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

	I2Cdev::writeByte(ADDRESS, CTRL1, 0x40); // Serial Interface and Sensor Enable<串行接口（SPI或I2C）地址自动递增>
	// I2Cdev::writeByte(ADDRESS, CTRL7, 0x03); // Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer>
	I2Cdev::writeByte(ADDRESS, CTRL6, 0x06); // Set AE ODR to 64hz
	I2Cdev::writeByte(ADDRESS, CTRL7, 0x0B); // Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer AttitudeEngine>
	I2Cdev::writeByte(ADDRESS, CTRL2, 0x14); // Accelerometer Settings<0x04: ±2g  500Hz> <0x14: ±4g  500Hz>
	I2Cdev::writeByte(ADDRESS, CTRL3, 0x54); // Gyroscope Settings<0x64 ±2048dps 500Hz> <0x54 ±512dps 500Hz>
	// I2Cdev::writeByte(ADDRESS, CTRL5, 0x11); // Sensor Data Processing Settings<Enable Gyroscope Accelerometer 低通滤波>
	I2Cdev::writeByte(ADDRESS, CTRL5, 0x15); // Gyro & Accel Low-Pass 0001 0101
	delay(2000);

	// 初始化校准
	cali.isCalibrated = false;
	return false;
}

void QMI8658::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	*ax = readBytes(cmd[0]);
	*ay = readBytes(cmd[1]);
	*az = readBytes(cmd[2]);
	*gx = readBytes(cmd[4]);
	*gy = readBytes(cmd[5]);
	*gz = readBytes(cmd[6]);
}

// 从AE中直接获取四元数
void QMI8658::getQuaternion(float *q1, float *q2, float *q3, float *q4)
{
	*q1 = readBytes(dQW_L);
	*q2 = readBytes(dQX_L);
	*q3 = readBytes(dQY_L);
	*q4 = readBytes(dQZ_L);
	Serial.printf("AE四元数原始输出:   [%d,%d,%d,%d]\n", readBytes(dQW_L), readBytes(dQX_L), readBytes(dQY_L), readBytes(dQZ_L));
	Serial.println(readBytes(dQX_L));
}

// 将四元数转换为欧拉角
void QMI8658::getEularAngle(float *roll, float *pitch, float *yaw)
{
	Quat q;
	float dqw, dqx, dqy, dqz;
	getQuaternion(&dqw, &dqx, &dqy, &dqz);
	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;
	// roll (x-axis rotation)
	float t0 = +2.0 * (dqw * dqx + dqy * dqz);
	float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
	*roll = atan2(t0, t1);
	// pitch (y-axis rotation)
	float t2 = +2.0 * (dqw * dqy - dqz * dqx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	*pitch = asin(t2);
	// yaw (z-axis rotation)
	float t3 = +2.0 * (dqw * dqz + dqx * dqy);
	float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
	*yaw = atan2(t3, t4);
}

// 获取转换后的六轴数值
void QMI8658::getScaledMotion6(float acc[3], float gyro[3])
{
	getAcceleration(&ax, &ay, &az);
	getRotation(&gx, &gy, &gz);
	acc[0] = (float)(ax * ONE_G) / SSVT_A;
	acc[1] = (float)(ay * ONE_G) / SSVT_A;
	acc[2] = (float)(az * ONE_G) / SSVT_A;
	gyro[0] = (float)(gx * PI) / (SSVT_G * 180);
	gyro[1] = (float)(gy * PI) / (SSVT_G * 180);
	gyro[2] = (float)(gz * PI) / (SSVT_G * 180);
}

void QMI8658::getAcceleration(int16_t *ax, int16_t *ay, int16_t *az)
{
	*ax = readBytes(cmd[0]);
	*ay = readBytes(cmd[1]);
	*az = readBytes(cmd[2]);
}

void QMI8658::getRotation(int16_t *gx, int16_t *gy, int16_t *gz)
{
	*gx = readBytes(cmd[4]);
	*gy = readBytes(cmd[5]);
	*gz = readBytes(cmd[6]);
}

uint8_t QMI8658::getDeviceID()
{
	uint8_t chip_id = 0x00;
	I2Cdev::readBytes(ADDRESS, WHO_AM_I, 1, &chip_id);
	return chip_id;
}

// 虚假的testConnection
bool QMI8658::testConnection()
{
	return getDeviceID();
}

// 校准
// TODO: 将校准函数迁移到qmi8658sensor.cpp中
void QMI8658::getCalibratedData(float acc[3], float gyro[3])
{
	float acc_raw[3];
	float gyro_raw[3];
	uint8_t axis = 0;

	// 获取单位转换后的六轴运动数据
	getScaledMotion6(acc_raw, gyro_raw);
	// 这两个变量需要修改为实际用的
	if (!cali.isCalibrated)
	{
		if (cali.caliNum == 0)
		{
			memset((void *)cali.accDataSum, 0, sizeof(cali.accDataSum));
			memset((void *)cali.gyroDataSum, 0, sizeof(cali.gyroDataSum));
		}
		else if (cali.caliNum < CALI_DATA_NUM)
		{
			for (axis = 0; axis < 3; axis++)
			{
				cali.gyroDataSum[axis] += gyro_raw[axis];
				if (axis == 2)
				{
					cali.accDataSum[axis] += (acc_raw[axis] - ONE_G);
				}
				else
				{
					cali.accDataSum[axis] += acc_raw[axis];
				}
			}
			// Serial.printf("加速度原始值:   [%2.4lf,%2.4lf,%2.4lf]\n", acc_raw[0], acc_raw[1], acc_raw[2]);
			// Serial.printf("陀螺仪原始值:   [%2.4lf,%2.4lf,%2.4lf]\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
		}
		else if (cali.caliNum == CALI_DATA_NUM)
		{
			// Serial.printf("加速度累计偏移值:   [%2.4lf,%2.4lf,%2.4lf]\n", cali.accDataSum[0], cali.accDataSum[1], cali.accDataSum[2]);
			// Serial.printf("陀螺仪累计偏移值:   [%2.4lf,%2.4lf,%2.4lf]\n", cali.gyroDataSum[0], cali.gyroDataSum[1], cali.gyroDataSum[2]);
			for (axis = 0; axis < 3; axis++)
			{
				cali.gyroOffset[axis] = (0.0f - (cali.gyroDataSum[axis] / (CALI_DATA_NUM - 1)));
				cali.accOffset[axis] = (0.0f - (cali.accDataSum[axis] / (CALI_DATA_NUM - 1)));
			}
			Serial.printf("加速度计偏移值：[%.5f, %.5f, %.5f]\n", cali.accOffset[0], cali.accOffset[1], cali.accOffset[2]);
			Serial.printf("陀螺仪偏移值：[%.5f, %.5f, %.5f]\n", cali.gyroOffset[0], cali.gyroOffset[1], cali.gyroOffset[2]);
			cali.isCalibrated = true;
		}
		// 校准次数加一
		cali.caliNum++;
		// 标志位，表示校准尚未结束
		acc[0] = -1.0f;
	}
	// 已校准，则输出校准后的数据
	else
	{
		for (axis = 0; axis < 3; axis++)
		{
			acc[axis] = acc_raw[axis] + cali.accOffset[axis];
			gyro[axis] = gyro_raw[axis] + cali.gyroOffset[axis];
		}
	}
}
// 获取温度，尚未编写
int16_t QMI8658::getTemperature()
{
	return 0;
}
