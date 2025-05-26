/*

// Who AM I - using Wire lib:

#include <Wire.h>

#define MPU6050_ADDR     0x68
#define WHO_AM_I_REG     0x75

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔍 Starting I2C scan and WHO_AM_I check...");

  // אתחול I2C עם פינים מותאמים (ל-ESP32 S3)
  Wire.begin(42, 41); // SDA, SCL

  // סריקה כדי לוודא שהרכיב מחובר
  Serial.println("Scanning for I2C devices...");
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
    delay(5);
  }

  // קריאה מהרגיסטר WHO_AM_I
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(WHO_AM_I_REG);
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom(MPU6050_ADDR, 1);

  if (Wire.available())
  {
    uint8_t value = Wire.read();
    Serial.printf("✅ WHO_AM_I = 0x%02X\n", value);
  }
  else
  {
    Serial.println("❌ Failed to read WHO_AM_I.");
  }
}

void loop()
{
  // לא עושה כלום בלופ
}*/








// my code

#define SDA                     (42)
#define SCL                     (41)
#define MPU6050_BASE_ADD        (0X68)
#define WHO_AM_I_REG            (0x75)
#define WRITE                   (0)
#define READ                    (1)
#define MPU6050_WRITE(address)  ((address << 1) | WRITE)
#define MPU6050_READ(address)   ((address << 1) | READ)        

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

uint8_t i2c_read_byte()
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
  digitalWrite(SDA, HIGH);
  i2c_delay();

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

  i2c_start();                                                // start condition

  bool ack = i2c_write_byte(MPU6050_WRITE(MPU6050_BASE_ADD)); // slave address + WRITE bit
  Serial.printf("ACK1=%d  (ADDR)\n", ack);
  if (ack == false)
  {
    Serial.println("no ack arrived. ERR 1.");
  }

  ack = i2c_write_byte(WHO_AM_I_REG);                         // register address
  Serial.printf("ACK2=%d  (WHO_AM_I)\n", ack);
  if (ack == false)
  {
    Serial.println("no ack arrived. ERR 2.");
  }

  i2c_start();                                                // Repeated Start condition
  ack = i2c_write_byte(MPU6050_READ(MPU6050_BASE_ADD));       // slave address + READ bit
  Serial.printf("ACK3=%d  (READ)\n", ack);
  if (ack == false)
  {
    Serial.println("no ack arrived. ERR 3.");
  }

  uint8_t value = i2c_read_byte();                            // Read 1 byte
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







