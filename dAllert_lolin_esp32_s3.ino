#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include "i2c.h"
#include "mpu6050.h"

#define SDA                     (42)
#define SCL                     (41)
#define DISABLE                 (0)  

Preferences preferences;
const char* telBotToken = "8084151876:AAHquYPcwWSh195GL-Rq5Vu6jjEgLMBBHec";
const char* chatId = "684325257"; 

void sendTelegramMessage(const String& message)
{
  if(WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    String url = "https://api.telegram.org/bot";
    url += telBotToken;
    url += "/sendMessage?chat_id=";
    url += chatId;
    url += "&text=";
    url += message;

    http.begin(url);
    int httpResponseCode = http.GET();
    http.end();

    if (httpResponseCode > 0)
      Serial.println("Message sent");
    else
      Serial.printf("Error sending message: %d\n", httpResponseCode);
  }
  else
  {
    Serial.println("WiFi not connected");
  }
}

void connectToWiFi()
{
  String ssid, password;

  preferences.begin("wifi", false);

  if (!preferences.isKey("ssid"))
  {
    preferences.putString("ssid", "spada");
    preferences.putString("password", "75395128ms");
    Serial.println("WiFi credentials saved to NVS.");
  }

  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");

  Serial.printf("Connecting to %s...\n", ssid.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long wifiTimeout = 30000; // 30 seconds
  unsigned long startTime = millis();

  while ((WiFi.status() != WL_CONNECTED) && ((millis() - startTime) < wifiTimeout) )
  {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.println("**************** Failed to connect to WiFi! **************** ");
  }
  else
  {
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
  }

  preferences.end();
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello from LOLIN S3!");

  i2c_init();
  delay(500);
  mpu6050_reset();
  mpu6050_set_clock();
  mpu6050_sleep_mode(DISABLE);
  mpu6050_enable_axes();
  mpu6050_set_gyro_range();
  delay(1000);
  mpu6050_calibrate_gyro();

  connectToWiFi();
}

void loop()
{
  mpu6050_read_gyro();

  if(abs(mpu6050_get_gyro_x()) > 10)
  {
    Serial.println("xxxxxxxxxxxxxxxxxxxxxxxx");
    sendTelegramMessage("motion detected! axis x");
  }
  if(abs(mpu6050_get_gyro_y()) > 10)
  {
    Serial.println("yyyyyyyyyyyyyyyyyyyyyyyy");
    sendTelegramMessage("motion detected! axis y");
  }
  if(abs(mpu6050_get_gyro_z()) > 10)
  {
    Serial.println("zzzzzzzzzzzzzzzzzzzzzzz");
    sendTelegramMessage("motion detected! axis z");
  }    
  delay(500);
}



