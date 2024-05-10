#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <UrlEncode.h>

const char* ssid = "Livemobile";
const char* password = "5432112345";
String phoneNumber = "+970597527564";
String apiKey = "8937783";

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;


void sendMessage(String message){

  // Data to send with HTTP POST
  String url = "https://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKey + "&text=" + urlEncode(message);    
  HTTPClient http;
  http.begin(url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200){
    Serial.print("Message sent successfully");
  }
  else{
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU-6050
  Wire.endTransmission(true);

  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  mpu_read();

  // Calculate acceleration amplitudes
  float ax = (AcX - 2050) / 16384.00;
  float ay = (AcY - 77) / 16384.00;
  float az = (AcZ - 1947) / 16384.00;

  // Calculate amplitude vector
  float raw_amplitude = sqrt(ax * ax + ay * ay + az * az);
  int amplitude = raw_amplitude * 10;  // Amplify for better sensitivity

  Serial.println(amplitude);

  // Check for fall condition
  if (amplitude <= 2) {  // If acceleration magnitude is below threshold
    // Implement more complex fall detection logic here
    sendMessage("FALL DETECTED!");
    delay(10000);  // Delay to prevent spamming messages
  }

  delay(100);  // Adjust delay for sensor reading frequency
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // Request a total of 14 registers

  AcX = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT_H & ACCEL_XOUT_L
  AcY = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT_H & ACCEL_YOUT_L
  AcZ = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT_H & ACCEL_ZOUT_L
  // Temperature
  Wire.read() << 8 | Wire.read();        // TEMP_OUT_H & TEMP_OUT_L
  GyX = Wire.read() << 8 | Wire.read();  // GYRO_XOUT_H & GYRO_XOUT_L
  GyY = Wire.read() << 8 | Wire.read();  // GYRO_YOUT_H & GYRO_YOUT_L
  GyZ = Wire.read() << 8 | Wire.read();  // GYRO_ZOUT_H & GYRO_ZOUT_L
}