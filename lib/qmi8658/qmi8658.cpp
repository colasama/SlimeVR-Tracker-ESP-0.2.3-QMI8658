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

void QMI8658::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  *ax = readBytes(cmd[0]);
  *ay = readBytes(cmd[1]);
  *az = readBytes(cmd[2]);
  *gx = readBytes(cmd[4]);
  *gy = readBytes(cmd[5]);
  *gz = readBytes(cmd[6]);
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
