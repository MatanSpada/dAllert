#define SDA                     (42)
#define SCL                     (41)
#define MPU6050_BASE_ADD        (0X68)
#define WHO_AM_I_REG            (0x75)
#define WRITE                   (0)
#define READ                    (1)
#define MPU5060_WRITE(address)  ((address << 1) | WRITE)
#define MPU5060_READ(address)   ((address << 1) | READ)        

void i2c_delay() {
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
  pinMode(SDA, INPUT);

  digitalWrite(SCL, HIGH);   
  i2c_delay();
  pinMode(SDA, OUTPUT);

  digitalWrite(SDA, LOW);
  i2c_delay();

  digitalWrite(SCL, LOW);
  i2c_delay();
}

void i2c_write_bit(bool bit)
{
  digitalWrite(SCL, LOW);
  i2c_delay(); // 1

  digitalWrite(SDA, bit);
  i2c_delay();

  digitalWrite(SCL, HIGH); // slave read
  i2c_delay(); // 5

  digitalWrite(SCL, LOW);
  i2c_delay(); // 2
}

bool i2c_read_ack()
{
  pinMode(SDA, INPUT);
  digitalWrite(SCL, HIGH);
  i2c_delay(); // 5

  bool ack = (digitalRead(SDA) == LOW);

  digitalWrite(SCL, LOW);
  pinMode(SDA, OUTPUT);

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

  pinMode(SDA, INPUT);

  return i2c_read_ack();
}

uint8_t i2c_read_byte()
{
  uint8_t value = 0;
  int i = 0;

  pinMode(SDA, INPUT);

  for (i = 0; i < 8; ++i)
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

  // end transmition and sending NACK
  digitalWrite(SCL, LOW);
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, HIGH); // NACK
  i2c_delay();

  digitalWrite(SCL, HIGH);
  i2c_delay();

  digitalWrite(SCL, LOW);
  i2c_delay();

  digitalWrite(SDA, HIGH); 
  pinMode(SDA, INPUT);

  return value;
}


void i2c_stop()
{
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, HIGH);
  i2c_delay();

  pinMode(SDA, INPUT);
  i2c_delay();
}



void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello from LOLIN S3!");

  i2c_init();
}


void loop()
{
  delay(1000);  

  i2c_start();

  bool ack = i2c_write_byte(MPU5060_WRITE(MPU6050_BASE_ADD));
  Serial.printf("ACK1=%d  (ADD)\n", ack);
  if(ack != true)
  {
    Serial.println("no ack arrived. ERR 1.");
  }

  ack = i2c_write_byte(WHO_AM_I_REG);
  Serial.printf("ACK2=%d  (WHO_AM_I)\n", ack);
  if(ack != true)
  {
    Serial.println("no ack arrived. ERR 2.");
  }

  i2c_start();
  ack = i2c_write_byte(MPU5060_READ(MPU6050_BASE_ADD));
  Serial.printf("ACK3=%d  (READ)\n", ack);
  if (ack != true)
  {
    Serial.println("no ack arrived. ERR 3.");
  }

  uint8_t value = i2c_read_byte();

  i2c_stop();


  Serial.printf("value: 0x%02X\n", value);

  Serial.println("delay");
  delay(1000);  
}


// #include <Wire.h>

// void setup()
// {
//   Serial.begin(115200);
//   delay(1000);


//   Wire.begin(42, 41);  

//   Serial.println("🔍 I2C Scanner");

//   for (uint8_t address = 1; address < 127; address++)
//   {
//     Wire.beginTransmission(address);
//     if (Wire.endTransmission() == 0)
//     {
//       Serial.print("Found device at 0x");
//       Serial.println(address, HEX);
//     }
//     delay(10);
//   }

//   Serial.println("****Scan complete.");
// }

// void loop()
// {
  
// }
