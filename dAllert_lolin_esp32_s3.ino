#define SDA                 (42)
#define SCL                 (41)
#define MPU5060_ADD         (0X69)
#define WRITE               (0)
#define READ                (1)
#define I2C_WRITE_ADD(address)  ((address << 1) | WRITE)
#define I2C_READ_ADD(address)   ((address << 1) | READ)        

void delayI2C() {
  delayMicroseconds(5); 
}

void i2c_init()
{
  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);
  digitalWrite(SDA, HIGH);
  digitalWrite(SCL, HIGH);
  delayI2C();
}

void i2c_start()
{
  digitalWrite(SDA, HIGH);   
  digitalWrite(SCL, HIGH);   
  delayI2C();

  digitalWrite(SDA, LOW);
  delayI2C();

  digitalWrite(SCL, LOW);
  delayI2C();
}

void i2c_write_bit(bool bit)
{
  digitalWrite(SCL, LOW);
  delayI2C(); // 1

  digitalWrite(SDA, bit);
  delayI2C(); // 2

  digitalWrite(SCL, HIGH); // slave read
  delayI2C(); // 5

  digitalWrite(SCL, LOW);
  delayI2C(); // 2
}

bool i2c_read_ack()
{
  pinMode(SDA, INPUT);
  digitalWrite(SCL, HIGH);
  delayI2C(); // 5

  bool ack = (digitalRead(SDA) == LOW);

  digitalWrite(SCL, LOW);
  pinMode(SDA, OUTPUT);

  return ack;
}

bool i2c_write_byte(uint8_t byte)
{
  uint8_t m = 1 << 7;

  while(m)
  {
    i2c_write_bit(byte & m);
    m >>= 1;
  }

  return i2c_read_ack();
}

void i2c_stop()
{
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  delayI2C();

  digitalWrite(SCL, HIGH);
  delayI2C();

  digitalWrite(SDA, HIGH);
  delayI2C();
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
  i2c_start();
  bool ack = i2c_write_byte(I2C_WRITE_ADD(MPU5060_ADD));
  i2c_stop();
  Serial.println(ack == true);
  Serial.println("delay");
  delay(1000); 
}


// #include <Wire.h>

// void setup()
// {
//   Serial.begin(115200);
//   delay(1000);

//   // הגדרת פינים של I2C (לפי החיבור שלך)
//   Wire.begin(42, 41);  // SDA = GPIO 8, SCL = GPIO 9

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
//   // לא צריך לולאה — סורק פעם אחת
// }
