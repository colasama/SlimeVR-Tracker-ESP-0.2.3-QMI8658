/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#ifndef SENSORS_QMI8658SENSOR_H
#define SENSORS_QMI8658SENSOR_H

#include "sensor.h"
#include "logging/Logger.h"

// #include "magneto1.4.h"//这个是什么算法，干啥的好复杂

#include "qmi8658.h"
#include <qmc5883l.h>

class QMI8658Sensor : public Sensor {
    public:
        QMI8658Sensor(uint8_t id, uint8_t address, float rotation) : Sensor("QMI8658Sensor", IMU_QMI8658, id, address, rotation){};
        ~QMI8658Sensor(){};
        void motionSetup() override final;
        void motionLoop() override final;
        void startCalibration(int calibrationType) override final;
        void getMPUScaled();
    private:
        QMI8658 imu {};
        QMC5883L mag;

        //抄9250的有些可能用不到
        uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
        uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
        uint16_t fifoCount;       // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]{}; // FIFO storage buffer
        
        // raw data and scaled as vector    原始数据并缩放为向量？
        float q[4] {1.0f, 0.0f, 0.0f, 0.0f};// for raw filter   过滤器？
        float Axyz[3]{};
        float Gxyz[3]{};
        float Mxyz[3]{};
        float rawMag[3]{};
        Quat correction{0, 0, 0, 0};

        // Loop timing globals
        unsigned long now = 0, last = 0; // micros() timers
        float deltat = 0;                // loop time in seconds

        SlimeVR::Configuration::QMI8658CalibrationConfig m_Calibration;//保存的校准数据
};

#endif
