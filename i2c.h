#ifndef I2C_H
#define I2C_H

void i2c_init();
void i2c_start();
void i2c_stop();
void i2c_delay();
bool i2c_read_ack();
void i2c_write_bit(bool bit);
bool i2c_write_byte(uint8_t byte);
uint8_t i2c_read_byte(bool send_ack);
void i2c_ack_check(bool ack, const char *msg);


#endif // #define I2C_H