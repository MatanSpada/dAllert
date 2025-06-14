#include <Arduino.h>
#include "mpu6050.h"
#include "i2c.h"

#define WRITE                   (0)
#define READ                    (1)
#define MPU6050_BASE_ADD        (0X68)
#define PWR_MGMT_1              (0x6B)
#define PWR_MGMT_2              (0x6C)
#define GYRO_CONFIG             (0x1B)
#define MPU6050_WRITE(address)  ((address << 1) | WRITE)
#define MPU6050_READ(address)   ((address << 1) | READ)    
#define ENABLE                  (1)
#define DISABLE                 (0)  
#define GYRO_XOUT_H             (0x43) // [15:8]
#define GYRO_XOUT_L             (0x44) // [7:0]
#define GYRO_YOUT_H             (0x45) // [15:8]
#define GYRO_YOUT_L             (0x46) // [7:0]
#define GYRO_ZOUT_H             (0x47) // [15:8]
#define GYRO_ZOUT_L             (0x48) // [7:0]

typedef enum 
{
  FS_SEL_0 = 0, // ±250 °/s
  FS_SEL_1,     // ±500 °/s
  FS_SEL_2,     // ±1000 °/s
  FS_SEL_3      // ±2000 °/s
} FS_SEL_VALUE;

FS_SEL_VALUE g_fs_sel_value = FS_SEL_0;
float sensitivity_table[4] = {131, 65.5, 32.8, 16.4};

bool is_calibration_g = false;
float gyro_bias_x_g = 0;
float gyro_bias_y_g = 0;
float gyro_bias_z_g = 0;
float gyro_x_g = 0;
float gyro_y_g = 0;
float gyro_z_g = 0;

void mpu6050_reset()
{
  i2c_start();  

  i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD)); // slave address + WRITE bit
  i2c_write_byte(PWR_MGMT_1);// register address ack);
  i2c_write_byte(0x80);// Reset;

  i2c_stop();

  delay(100);
}

void mpu6050_set_clock()
{
  i2c_start();

  bool ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD));
  i2c_ack_check(ack, "set_clock: write slave address");

  ack = i2c_write_byte(PWR_MGMT_1);
  i2c_ack_check(ack, "set clock: write register");

  ack = i2c_write_byte(0x01); // Set PLL with X gyro
  i2c_ack_check(ack, "set clock: write value");

  i2c_stop();

  delay(100);
}

void mpu6050_sleep_mode(uint8_t state) // enable = 1, disable = 0
{
  uint8_t reg_value = 0;

  // read current register state
  i2c_start();

  bool ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD));
  i2c_ack_check(ack, "sleep_mode: write slave address");

  ack = i2c_write_byte(PWR_MGMT_1);
  i2c_ack_check(ack, "sleep_mode: write register");

  i2c_start();
  ack = i2c_write_byte(MPU6050_READ(MPU6050_BASE_ADD)); // read
  i2c_ack_check(ack, "read for sleep: read slave address");
  reg_value = i2c_read_byte(0);
  i2c_stop();

  if(state == ENABLE)
  {
    reg_value |= (1 << 6); // enable
  }
  else // disable
  {
    reg_value &= ~(1 << 6);
  }

  // update register state
  i2c_start();

  ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD));
  i2c_ack_check(ack, "write for sleep: write slave address");

  ack = i2c_write_byte(PWR_MGMT_1);
  i2c_ack_check(ack, "write for sleep: write register");

  ack = i2c_write_byte(reg_value);
  i2c_ack_check(ack, "write for sleep: write new value");

  i2c_stop();
}

void mpu6050_enable_axes()
{
  i2c_start();

  bool ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD));
  i2c_ack_check(ack, "enable_axes: write slave address");

  ack = i2c_write_byte(PWR_MGMT_2); // PWR_MGMT_2
  i2c_ack_check(ack, "enable_axes: write register");

  ack = i2c_write_byte(0x00); // enable all axes gyro and accel
  i2c_ack_check(ack, "enable_axes: write value");

  i2c_stop();
}


void mpu6050_set_gyro_range()
{
  uint8_t range_value = 0;
  uint8_t reg_value = 0;

  switch (g_fs_sel_value)
  {
    case 0: range_value = 0x00; break; 
    case 1: range_value = 0x08; break; 
    case 2: range_value = 0x10; break; 
    case 3: range_value = 0x18; break; 
    default: return;
  }


  // read current register state
  i2c_start();

  bool ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD));
  i2c_ack_check(ack, "set_gyro_range: write slave address");

  ack = i2c_write_byte(GYRO_CONFIG); // GYRO_CONFIG REG
  i2c_ack_check(ack, "set_gyro_range: write register");

  i2c_start();
  ack = i2c_write_byte(MPU6050_READ(MPU6050_BASE_ADD)); // read
  i2c_ack_check(ack, "set_gyro_range: read slave address");
  reg_value = i2c_read_byte(0);

  i2c_stop();

  // update only bits 4:3 (FS_SEL)
  reg_value &= ~(0x18);         // clear bits 4 and 3
  reg_value |= range_value;     // set new FS_SEL value

  // write back updated register value
  i2c_start();
  ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD));
  i2c_ack_check(ack, "gyro_range: write slave address");

  ack = i2c_write_byte(GYRO_CONFIG); // GYRO_CONFIG REG
  i2c_ack_check(ack, "gyro_range: write register");

  ack = i2c_write_byte(reg_value);
  i2c_ack_check(ack, "gyro_range: write value");

  i2c_stop();
}

void mpu6050_read_gyro()
{
  float sensitivity = 0;
  uint8_t axes_value[6] = { 0 };
  int16_t gyro_x_raw = 0;
  int16_t gyro_y_raw = 0;
  int16_t gyro_z_raw = 0;

  i2c_start();
  
  bool ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD));
  i2c_ack_check(ack, "read_gyro: write slave address");

  ack = i2c_write_byte(GYRO_XOUT_H);
  i2c_ack_check(ack, "read_gyro: write register");

  i2c_start();
  ack = i2c_write_byte(MPU6050_READ(MPU6050_BASE_ADD)); // read command in packet
  i2c_ack_check(ack, "read_gyro: read register");

  for(int i = 0; i < 6; ++i)
  {
    axes_value[i] = i2c_read_byte(i < 5); // ACK to 0-4, NACK to 5
  }

  sensitivity = sensitivity_table[g_fs_sel_value];

  gyro_x_raw = (int16_t)((axes_value[0] << 8) | axes_value[1]); // (int16_t) is essential due to Integer Promotion behavior:
  gyro_y_raw = (int16_t)((axes_value[2] << 8) | axes_value[3]); // the result of (uint8_t << 8) is promoted to int (or unsigned int), 
  gyro_z_raw = (int16_t)((axes_value[4] << 8) | axes_value[5]); // which may misinterpret the sign bit if not explicitly cast.
  
  if(is_calibration_g == true)
  {
    gyro_x_g = (gyro_x_raw / sensitivity); 
    gyro_y_g = (gyro_y_raw / sensitivity);
    gyro_z_g = (gyro_z_raw / sensitivity);
  }
  else
  {
    gyro_x_g = (gyro_x_raw / sensitivity) - gyro_bias_x_g; 
    gyro_y_g = (gyro_y_raw / sensitivity) - gyro_bias_y_g;
    gyro_z_g = (gyro_z_raw / sensitivity) - gyro_bias_z_g;    

    // Serial.printf("Gyro X: %.2f °/s\n", gyro_x_g);
    // Serial.printf("Gyro Y: %.2f °/s\n", gyro_y_g);
    // Serial.printf("Gyro Z: %.2f °/s\n", gyro_z_g);
  }


  i2c_stop();
}


void mpu6050_calibrate_gyro()
{
  const int num_samples = 1000;

  Serial.println("Gyro is calibrating and will start automatically...");
  is_calibration_g = true;
  
  for(int i = 0; i < num_samples; ++i)
  {
    mpu6050_read_gyro();
    gyro_bias_x_g += gyro_x_g;
    gyro_bias_y_g += gyro_y_g;
    gyro_bias_z_g += gyro_z_g;
    delay(10);
  }

  Serial.println("Done calibrating.");
  is_calibration_g = false;

  gyro_bias_x_g = gyro_bias_x_g / num_samples;
  gyro_bias_y_g = gyro_bias_y_g / num_samples;
  gyro_bias_z_g = gyro_bias_z_g / num_samples;
}

float mpu6050_get_gyro_x()
{
  return gyro_x_g;
}

float mpu6050_get_gyro_y()
{
  return gyro_y_g;
}

float mpu6050_get_gyro_z()
{
  return gyro_z_g;
}