#include <Arduino.h>
#include "i2c.h"


void i2c_delay() 
{
  delayMicroseconds(20); 
}

void i2c_init()
{
  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);
  digitalWrite(SDA, HIGH);
  digitalWrite(SCL, HIGH);
  i2c_delay();
}

void i2c_start()
{
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  i2c_delay();

  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  i2c_delay();

  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, LOW);
  i2c_delay();
}

void i2c_write_bit(bool bit)
{
  digitalWrite(SCL, LOW);
  i2c_delay();

  digitalWrite(SDA, bit);

  i2c_delay();

  digitalWrite(SCL, HIGH);
  i2c_delay();

  digitalWrite(SCL, LOW);
  i2c_delay();
}

bool i2c_read_ack()
{
  pinMode(SDA, INPUT_PULLUP);
  digitalWrite(SCL, HIGH);
  i2c_delay();

  bool ack = (digitalRead(SDA) == LOW);

  digitalWrite(SCL, LOW);
  return ack;
}

bool i2c_write_byte(uint8_t byte)
{
  pinMode(SDA, OUTPUT);
  i2c_delay();

  uint8_t m = 1 << 7;
  while(m)
  {
    i2c_write_bit(byte & m);
    m >>= 1;
  }

  return i2c_read_ack();
}

uint8_t i2c_read_byte(bool send_ack)
{
  uint8_t value = 0;
  pinMode(SDA, INPUT_PULLUP);

  for (int i = 0; i < 8; ++i)
  {
    digitalWrite(SCL, LOW);
    i2c_delay();

    digitalWrite(SCL, HIGH);
    i2c_delay();

    value <<= 1;
    if (digitalRead(SDA))
    {
      value |= 1;
    }
  }

  // Send NACK
  digitalWrite(SCL, LOW);
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, send_ack != 0 ? LOW : HIGH);   // ACK=0, NACK=1  i2c_delay();

  digitalWrite(SCL, HIGH);
  i2c_delay();

  digitalWrite(SCL, LOW);
  i2c_delay();

  pinMode(SDA, INPUT_PULLUP);

  return value;
}

void i2c_stop()
{
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, HIGH);
  i2c_delay();

  pinMode(SDA, INPUT_PULLUP);
  i2c_delay();
}

void i2c_ack_check(bool ack, const char *msg)
{
  if (ack == false)
  {
    Serial.printf("no ack arrived. ERR: %s.\n", msg);
    //while(1);
  }
}