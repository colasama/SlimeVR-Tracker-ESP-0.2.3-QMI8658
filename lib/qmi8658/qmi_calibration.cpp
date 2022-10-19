
void performCalibration(float acc[3], float gyro[3]) {
    float           acc_raw[3];
    float           gyro_raw[3];
    uint8_t         axis = 0;
    union byte_float {
        float   float_data;
        uint8_t byte[4];
    } float_to_byte;

    // 获取单位转换后的六轴运动数据
    imu.getScaledMotion6(acc_raw[3], gyro_raw[3]);
    // 这两个变量需要修改为实际用的
    if(imu_init != 1) {
        if(cali_count == 0) {
            memset((void *)accel_calibration_sum, 0, sizeof(accel_calibration_sum));
            memset((void *)gyro_calibration_sum, 0, sizeof(gyro_calibration_sum));
            cali_count++;
        }
        else if(cali_count < MAX_CALI_COUNT) {
            for(axis = 0; axis < 3; axis++) {
                gyro_calibration_sum[axis] += gyro_raw[axis];
                if(axis == 2) {
                    accel_calibration_sum[axis] += (acc_raw[axis] - ONE_G);
                }
                else {
                    accel_calibration_sum[axis] += acc_raw[axis];
                }
            }
            cali_count++;
        }
        else if(cali_count == MAX_CALI_COUNT) {
            for(axis = 0; axis < 3; axis++) {
                offset_gyro[axis] = (gyro_calibration_sum[axis] / (MAX_CALI_COUNT - 1));
                offset_acc[axis] = (acc_calibration_sum[axis] / (MAX_CALI_COUNT - 1));
            }
            imu_init = 1;

            float_to_byte.float_data = offset_gyro[0];
            Calibration_data[0] = float_to_byte.byte[0]
        }
    }
}