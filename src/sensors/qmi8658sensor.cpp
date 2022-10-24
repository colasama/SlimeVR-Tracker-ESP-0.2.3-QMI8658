/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "qmi8658sensor.h"
#include "network/network.h"
#include "globals.h"
#include "helper_3dmath.h"
#include "calibration.h"
#include "magneto1.4.h"
#include "GlobalVars.h"

#include "mahony.h"//这两个融合算法，使用哪个就把另一个注释掉
// #include "madgwick.h"

// Typical sensitivity at 25C
// See p. 9 of https://www.mouser.com/datasheet/2/783/BST-QMI8658-DS000-1509569.pdf
// 65.6 LSB/deg/s = 500 deg/s
#define TYPICAL_SENSITIVITY_LSB 65.6
#define ONE_G			(9.807f)

#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float gscale = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);//好像是陀螺仪量程，后面校准要用的
#endif

// LSB change per temperature step map.
// These values were calculated for 500 deg/s sensitivity
// Step quantization - 5 degrees per step


// register address of data
const uint8_t cmd[] = {AccX_L, AccY_L, AccZ_L, TEMP_L, GyrX_L, GyrY_L, GyrZ_L};

void QMI8658Sensor::motionSetup() {
    // initialize device
    // imu.initialize(addr);
    imu.initialize();
    mag.initialize();
    if(!imu.testConnection()) {
        m_Logger.fatal("Can't connect to QMI8658 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to QMI8658 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

    // //TODO:给磁力计也加一个driver id校验
    // if(!mag.testConnection()) {
    //     m_Logger.fatal("Can't connect to QMC5883L (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
    //     ledManager.pattern(50, 50, 200);
    //     return;
    // }

    // m_Logger.info("Connected to QMC5883L (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
    
    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / 8192; // For 4G sensitivity
    m_Logger.info("G_az: %02f", g_az);
    if(g_az < -0.75f) {
        ledManager.on();

        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / 8192;
        if(g_az > 0.75f)
        {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }

        ledManager.off();
    }

    // Initialize the configuration 初始化校准
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        //如果没有找到兼容的校准数据，则校准数据将被置零    ↑
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::QMI8658:
            m_Calibration = sensorCalibration.data.qmi8658;
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }

    working = true;
}

void QMI8658Sensor::motionLoop() {
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
#else

    unsigned long now = micros();
    unsigned long deltat = now - last; //seconds since last update
    last = now;
    getMPUScaled();
    
    #if defined(_MAHONY_H_)
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    #elif defined(_MADGWICK_H_)
    madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    #endif
    quaternion.set(-q[2], q[1], q[3], q[0]);
#endif
    quaternion *= sensorOffset;


    // imu.getCalibratedData(Axyz, Gxyz);
    // while(Axyz[0] == -1.0f) {
    //     imu.getCalibratedData(Axyz, Gxyz);
    // }
    // // // 调试信息，先扔这里了
    // // Serial.printf("处理数据：\n");
    // Serial.printf("\n");
    // Serial.printf("测量加速度(m/S^2):   [%2.4lf,%2.4lf,%2.4lf]\n", Axyz[0], Axyz[1], Axyz[2]);
    // Serial.printf("测量角速度(deg/s):   [%.4lf,%.4lf,%.4lf]\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    // float qw, qx, qy, qz, er, ep, ey;
    // imu.getQuaternion(&qw, &qx, &qy, &qz);
    // imu.getEularAngle(&er, &ep, &ey);
    // Serial.printf("AE四元数:   [%.4lf,%.4lf,%.4lf,%.4lf]\n", qw, qx, qy, qz);
    // Serial.printf("AE欧拉角:   [%.2lf,%.2lf,%.2lf]\n",er, ep ,ey);
    // delay(400);

    /* 磁强度测试 */
    // int16_t mxt, myt, mzt;
    // mag.getMagetometerData(&mxt, &myt, &mzt);

    // Serial.printf("测量磁强度(Gauss):   [%.3lf,%.3lf,%.3lf]\n", mxt/1500.0, myt/1500.0, mzt/1500.0);
    // Serial.printf("提示：通常地磁场的强度是0.4-0.6 Gauss。\n");
    // delay(400);

    // //衡量磁感应强度大小的单位是Tesla或者Gauss（1Tesla=10000Gauss）。
    // //随着地理位置的不同，通常地磁场的强度是0.4-0.6 Gauss。
    // //量程2Guass的时候，增益系数为12 000，那么磁场值为hpx/12000（Guass）
    // //量程8Guass的时候，增益系数为1500，那么磁场值为hpx/1500（Guass）   大概？
    // //注意，磁场强度的单位为A/m，在空气中，A/m和高斯的转换关系为1高斯=79.62A/m，可以继续转换为磁场强度作为单位。
    // mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6f);
    // quaternion.set(-q[2], q[1], q[3], q[0]);
    // quaternion *= sensorOffset;

#if ENABLE_INSPECTION
    {
        Network::sendInspectionFusedIMUData(sensorId, quaternion);
    }
#endif

    if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }
}


/*
获取校准零偏后的数值
仿照9250的完全校准方法，删除了使用dmp的if（没啥用）
*/
void QMI8658Sensor::getMPUScaled()
{
    float temp[3];
    int i;

#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    //TODO:qmi8658还没有加磁力计，目前没有getMotion9这个函数
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getMagetometerData(&mx, &my, &mz);
    Gxyz[0] = ((float)gx - m_Calibration.G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - m_Calibration.G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - m_Calibration.G_off[2]) * gscale;

    //TODO:不校准加速度计零偏吗？
    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true    //使用完整的校准矩阵
        for (i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
        Axyz[0] = m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2];
        Axyz[1] = m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2];
        Axyz[2] = m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2];
    #else
        for (i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - m-Calibration.A_B[i]);
    #endif
}

/*
校准函数
仿照9250，删除了dmp部分
*/
void QMI8658Sensor::startCalibration(int calibrationType) {
    ledManager.on();

    m_Logger.debug("Gathering raw data for device calibration...");//为设备收集原始数据
    constexpr int calibrationSamples = 300;
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration  待传感器静下来后再校准
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");//放下设备，等待陀螺仪读数校准
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        mag.getMagetometerData(&mx, &my, &mz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;

#ifdef DEBUG_SENSOR
    m_Logger.trace("Gyro calibration results: %f %f %f", Gxyz[0], Gxyz[1], Gxyz[2]);
#endif

    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    m_Calibration.G_off[0] = Gxyz[0];
    m_Calibration.G_off[1] = Gxyz[1];
    m_Calibration.G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor 闪烁校准led，用户应旋转传感器
    m_Logger.info("Gently rotate the device while it's gathering accelerometer and magnetometer data");//轻轻地旋转设备，同时它正在收集加速度计和磁力计数据
    ledManager.pattern(15, 300, 3000/310);
    float *calibrationDataAcc = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        mag.getMagetometerData(&mx, &my, &mz);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        calibrationDataMag[i * 3 + 0] = my;
        calibrationDataMag[i * 3 + 1] = mx;
        calibrationDataMag[i * 3 + 2] = -mz;
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");//计算校准数据……

    float A_BAinv[4][3];
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);
    m_Logger.debug("Finished Calculate Calibration data");//完成计算校准数据
    m_Logger.debug("Accelerometer calibration matrix:");//加速度计校准矩阵:
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++)
    {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");
    m_Logger.debug("[INFO] Magnetometer calibration matrix:");//磁力计校准矩阵
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");
#endif

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::QMI8658;
    calibration.data.qmi8658 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");//校准数据已收集
}