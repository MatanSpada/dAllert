#ifndef MPU6050_H
#define MPU6050_H

void mpu6050_reset();
void mpu6050_read_gyro();
void mpu6050_set_clock();
void mpu6050_enable_axes();
void mpu6050_set_gyro_range();
void mpu6050_calibrate_gyro();
void mpu6050_sleep_mode(uint8_t state);

float mpu6050_get_gyro_x();
float mpu6050_get_gyro_y();
float mpu6050_get_gyro_z();

#endif // #define MPU6050_H