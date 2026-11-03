
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <time.h>
#include <math.h>

// =====================================================
// 1. WIFI AND MQTT CONFIGURATION
// =====================================================

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_SERVER = "192.168.1.37";
const int MQTT_PORT = 1883;

const char* MQTT_USER = "smartbox_device";
const char* MQTT_PASSWORD = "YOUR_MQTT_PASSWORD";

const char* MQTT_TOPIC = "smartbox/telemetry";

const char* DEVICE_ID = "esp32-box-001";
const char* JOURNEY_ID = "journey-001";

// =====================================================
// 2. PIN CONFIGURATION
// =====================================================

// MPU-compatible sensor: I2C
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDRESS 0x68

// DS18B20
#define DS18B20_PIN 4

// ST7789 TFT
#define TFT_CS   5
#define TFT_RST  16
#define TFT_DC   2
#define TFT_SCLK 18
#define TFT_MOSI 23

#define TFT_WIDTH 240
#define TFT_HEIGHT 240

// =====================================================
// 3. MPU REGISTER MAP
// =====================================================

#define MPU_WHO_AM_I       0x75
#define MPU_PWR_MGMT_1     0x6B
#define MPU_PWR_MGMT_2     0x6C
#define MPU_SMPLRT_DIV     0x19
#define MPU_CONFIG         0x1A
#define MPU_GYRO_CONFIG    0x1B
#define MPU_ACCEL_CONFIG   0x1C
#define MPU_ACCEL_XOUT_H   0x3B

// =====================================================
// 4. SENSOR CONFIGURATION
// =====================================================

// Accelerometer: ±8 g
// 4096 raw counts = 1 g
const float ACCEL_SCALE = 4096.0;

// Gyroscope: ±500 degrees/second
// 65.5 raw counts = 1 degree/second
const float GYRO_SCALE = 65.5;

// Gravity estimation
const float GRAVITY_ALPHA = 0.02;

// =====================================================
// 5. TIMING
// =====================================================

const unsigned long MOTION_INTERVAL = 20;       // 50 Hz
const unsigned long SENSOR_INTERVAL = 1000;
const unsigned long DISPLAY_INTERVAL = 250;
const unsigned long MQTT_INTERVAL = 5000;

const unsigned long WIFI_RETRY_INTERVAL = 10000;
const unsigned long MQTT_RETRY_INTERVAL = 5000;
const unsigned long MPU_RETRY_INTERVAL = 5000;

// DS18B20 conversion time at 10-bit resolution
const unsigned long TEMP_CONVERSION_TIME = 200;

// =====================================================
// 6. OBJECTS
// =====================================================

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

OneWire oneWire(DS18B20_PIN);
DallasTemperature temperatureSensor(&oneWire);

Adafruit_ST7789 tft(
  TFT_CS,
  TFT_DC,
  TFT_MOSI,
  TFT_SCLK,
  TFT_RST
);

// =====================================================
// 7. GLOBAL VARIABLES
// =====================================================

// Temperature
float temperatureC = NAN;
bool temperatureAvailable = false;
bool temperatureConversionPending = false;
unsigned long temperatureRequestTime = 0;

// Motion sensor
bool mpuAvailable = false;
uint8_t mpuIdentity = 0;

// Raw motion data
int16_t rawAccelX = 0;
int16_t rawAccelY = 0;
int16_t rawAccelZ = 0;

int16_t rawGyroX = 0;
int16_t rawGyroY = 0;
int16_t rawGyroZ = 0;

// Converted acceleration in g
float accelX = 0;
float accelY = 0;
float accelZ = 0;

// Converted gyroscope in degrees/second
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

// Gyroscope calibration offsets
float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

// Estimated gravity
float gravityX = 0;
float gravityY = 0;
float gravityZ = 0;

bool gravityInitialized = false;

// Vibration
float vibrationG = 0;
float vibrationPeakG = 0;

float vibrationSumSquares = 0;
float vibrationWindowPeak = 0;

unsigned long vibrationSampleCount = 0;

// Network and time
bool timeConfigured = false;
bool wifiAttempted = false;
bool mqttAttempted = false;

unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastMpuAttempt = 0;

// Task timers
unsigned long lastMotionSample = 0;
unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastVibrationWindow = 0;

// =====================================================
// 8. LOW-LEVEL MPU I2C FUNCTIONS
// =====================================================

bool readMPURegister(
  uint8_t reg,
  uint8_t &value
) {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);

  uint8_t error = Wire.endTransmission(false);

  if (error != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDRESS, (uint8_t)1) != 1) {
    return false;
  }

  if (!Wire.available()) {
    return false;
  }

  value = Wire.read();

  return true;
}

// -----------------------------------------------------

bool writeMPURegister(
  uint8_t reg,
  uint8_t value
) {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

// -----------------------------------------------------

int16_t combineBytes(
  uint8_t highByte,
  uint8_t lowByte
) {

  return (int16_t)(
    ((uint16_t)highByte << 8) | lowByte
  );
}

// =====================================================
// 9. MPU SENSOR INITIALIZATION
// =====================================================

bool initializeMPU() {

  Serial.println();
  Serial.println("Initializing motion sensor...");

  uint8_t identity = 0;

  if (!readMPURegister(MPU_WHO_AM_I, identity)) {

    Serial.println("Cannot read WHO_AM_I.");
    return false;
  }

  Serial.print("WHO_AM_I: 0x");
  Serial.println(identity, HEX);

  // Accept the observed identity and standard MPU6050 identity.
  // This does not independently prove the exact chip model.

  if (identity != 0x68 && identity != 0x70) {

    Serial.println("Unsupported sensor identity.");
    return false;
  }

  mpuIdentity = identity;

  // Reset sensor
  if (!writeMPURegister(MPU_PWR_MGMT_1, 0x80)) {

    Serial.println("Sensor reset command failed.");
    return false;
  }

  delay(100);

  // Select PLL clock and wake sensor
  if (!writeMPURegister(MPU_PWR_MGMT_1, 0x01)) {

    Serial.println("Cannot configure sensor clock.");
    return false;
  }

  // Enable all accelerometer and gyroscope axes
  if (!writeMPURegister(MPU_PWR_MGMT_2, 0x00)) {

    Serial.println("Cannot enable sensor axes.");
    return false;
  }

  delay(100);

  // Digital low-pass filter
  if (!writeMPURegister(MPU_CONFIG, 0x03)) {
    Serial.println("CONFIG write failed.");
    return false;
  }

  // Sample rate:
  // 1 kHz / (1 + 9) = 100 Hz
  if (!writeMPURegister(MPU_SMPLRT_DIV, 9)) {
    Serial.println("Sample-rate configuration failed.");
    return false;
  }

  // Gyroscope ±500 degrees/second
  if (!writeMPURegister(MPU_GYRO_CONFIG, 0x08)) {
    Serial.println("Gyroscope configuration failed.");
    return false;
  }

  // Accelerometer ±8 g
  if (!writeMPURegister(MPU_ACCEL_CONFIG, 0x10)) {
    Serial.println("Accelerometer configuration failed.");
    return false;
  }

  delay(100);

  // Verify configuration
  uint8_t gyroConfig;
  uint8_t accelConfig;
  uint8_t powerConfig;

  if (!readMPURegister(MPU_GYRO_CONFIG, gyroConfig) ||
      !readMPURegister(MPU_ACCEL_CONFIG, accelConfig) ||
      !readMPURegister(MPU_PWR_MGMT_1, powerConfig)) {

    Serial.println("Configuration verification failed.");
    return false;
  }

  Serial.print("GYRO_CONFIG: 0x");
  Serial.println(gyroConfig, HEX);

  Serial.print("ACCEL_CONFIG: 0x");
  Serial.println(accelConfig, HEX);

  Serial.print("PWR_MGMT_1: 0x");
  Serial.println(powerConfig, HEX);

  if ((gyroConfig & 0x18) != 0x08 ||
      (accelConfig & 0x18) != 0x10 ||
      (powerConfig & 0x40) != 0) {

    Serial.println("Sensor configuration verification failed.");
    return false;
  }

  gravityInitialized = false;

  Serial.println("Motion sensor initialized.");

  return true;
}

// =====================================================
// 10. READ RAW MOTION DATA
// =====================================================

bool readRawMotion() {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // Read 14 consecutive bytes:
  // Accel X/Y/Z, temperature, gyro X/Y/Z

  if (Wire.requestFrom(MPU_ADDRESS, (uint8_t)14) != 14) {
    return false;
  }

  uint8_t data[14];

  for (int i = 0; i < 14; i++) {

    if (!Wire.available()) {
      return false;
    }

    data[i] = Wire.read();
  }

  rawAccelX = combineBytes(data[0], data[1]);
  rawAccelY = combineBytes(data[2], data[3]);
  rawAccelZ = combineBytes(data[4], data[5]);

  rawGyroX = combineBytes(data[8], data[9]);
  rawGyroY = combineBytes(data[10], data[11]);
  rawGyroZ = combineBytes(data[12], data[13]);

  return true;
}

// =====================================================
// 11. GYROSCOPE CALIBRATION
// =====================================================

bool calibrateGyroscope() {

  Serial.println();
  Serial.println("GYROSCOPE CALIBRATION");
  Serial.println("Keep the box completely still.");

  const int samples = 200;

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  int validSamples = 0;

  for (int i = 0; i < samples; i++) {

    if (readRawMotion()) {

      sumX += rawGyroX;
      sumY += rawGyroY;
      sumZ += rawGyroZ;

      validSamples++;
    }

    delay(5);
  }

  if (validSamples < 150) {

    Serial.println("Calibration failed: insufficient samples.");
    return false;
  }

  gyroOffsetX = (float)sumX / validSamples;
  gyroOffsetY = (float)sumY / validSamples;
  gyroOffsetZ = (float)sumZ / validSamples;

  Serial.println("Gyroscope calibration completed.");

  Serial.print("Offset X: ");
  Serial.println(gyroOffsetX, 2);

  Serial.print("Offset Y: ");
  Serial.println(gyroOffsetY, 2);

  Serial.print("Offset Z: ");
  Serial.println(gyroOffsetZ, 2);

  return true;
}

// =====================================================
// 12. CONVERT MOTION DATA
// =====================================================

void convertMotionData() {

  accelX = rawAccelX / ACCEL_SCALE;
  accelY = rawAccelY / ACCEL_SCALE;
  accelZ = rawAccelZ / ACCEL_SCALE;

  gyroX = (rawGyroX - gyroOffsetX) / GYRO_SCALE;
  gyroY = (rawGyroY - gyroOffsetY) / GYRO_SCALE;
  gyroZ = (rawGyroZ - gyroOffsetZ) / GYRO_SCALE;
}

// =====================================================
// 13. MOTION SAMPLING AND VIBRATION
// =====================================================

void readMotion() {

  if (!mpuAvailable) {
    return;
  }

  if (!readRawMotion()) {

    Serial.println("Motion sensor read failed.");

    mpuAvailable = false;
    return;
  }

  convertMotionData();

  if (!gravityInitialized) {

    gravityX = accelX;
    gravityY = accelY;
    gravityZ = accelZ;

    gravityInitialized = true;

    return;
  }

  // Estimate gravity using a low-pass filter
  gravityX += GRAVITY_ALPHA * (accelX - gravityX);
  gravityY += GRAVITY_ALPHA * (accelY - gravityY);
  gravityZ += GRAVITY_ALPHA * (accelZ - gravityZ);

  // Remove estimated gravity
  float dynamicX = accelX - gravityX;
  float dynamicY = accelY - gravityY;
  float dynamicZ = accelZ - gravityZ;

  // Dynamic acceleration magnitude
  float dynamicMagnitude = sqrt(
    dynamicX * dynamicX +
    dynamicY * dynamicY +
    dynamicZ * dynamicZ
  );

  // Accumulate squared acceleration for RMS
  vibrationSumSquares +=
    dynamicMagnitude * dynamicMagnitude;

  vibrationSampleCount++;

  // Window peak
  if (dynamicMagnitude > vibrationWindowPeak) {

    vibrationWindowPeak = dynamicMagnitude;
  }
}

// -----------------------------------------------------

void calculateVibration() {

  unsigned long now = millis();

  if (now - lastVibrationWindow < 1000) {
    return;
  }

  lastVibrationWindow = now;

  if (vibrationSampleCount > 0) {

    vibrationG = sqrt(
      vibrationSumSquares / vibrationSampleCount
    );

    vibrationPeakG = vibrationWindowPeak;
  }

  vibrationSumSquares = 0;
  vibrationSampleCount = 0;
  vibrationWindowPeak = 0;
}

// =====================================================
// 14. DS18B20 TEMPERATURE SENSOR
// =====================================================

void startTemperatureConversion() {

  temperatureSensor.requestTemperatures();

  temperatureRequestTime = millis();
  temperatureConversionPending = true;
}

// -----------------------------------------------------

void updateTemperature() {

  if (!temperatureConversionPending) {
    startTemperatureConversion();
    return;
  }

  if (millis() - temperatureRequestTime <
      TEMP_CONVERSION_TIME) {

    return;
  }

  float value = temperatureSensor.getTempCByIndex(0);

  if (value == DEVICE_DISCONNECTED_C ||
      isnan(value) ||
      value < -40 ||
      value > 85) {

    temperatureAvailable = false;

    Serial.println("DS18B20 unavailable.");

  } else {

    temperatureC = value;
    temperatureAvailable = true;
  }

  startTemperatureConversion();
}

// =====================================================
// 15. WIFI CONNECTION
// =====================================================

void connectWiFi() {

  if (WiFi.status() == WL_CONNECTED) {

    if (!timeConfigured) {

      configTime(
        0,
        0,
        "pool.ntp.org",
        "time.nist.gov"
      );

      timeConfigured = true;

      Serial.println("UTC time synchronization started.");
    }

    return;
  }

  unsigned long now = millis();

  if (wifiAttempted &&
      now - lastWifiAttempt < WIFI_RETRY_INTERVAL) {

    return;
  }

  wifiAttempted = true;
  lastWifiAttempt = now;

  Serial.println("Connecting to Wi-Fi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// =====================================================
// 16. MQTT CONNECTION
// =====================================================

void connectMQTT() {

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (mqttClient.connected()) {
    return;
  }

  unsigned long now = millis();

  if (mqttAttempted &&
      now - lastMqttAttempt < MQTT_RETRY_INTERVAL) {

    return;
  }

  mqttAttempted = true;
  lastMqttAttempt = now;

  Serial.println("Connecting to MQTT...");

  if (mqttClient.connect(
        DEVICE_ID,
        MQTT_USER,
        MQTT_PASSWORD
      )) {

    Serial.println("MQTT connected!");

  } else {

    Serial.print("MQTT connection failed. State: ");
    Serial.println(mqttClient.state());
  }
}

// =====================================================
// 17. UTC TIMESTAMP
// =====================================================

String getTimestamp() {

  struct tm timeInfo;

  if (!getLocalTime(&timeInfo, 10)) {
    return "";
  }

  // Reject an unsynchronized system clock
  if (timeInfo.tm_year < (2024 - 1900)) {
    return "";
  }

  char timestamp[30];

  strftime(
    timestamp,
    sizeof(timestamp),
    "%Y-%m-%dT%H:%M:%SZ",
    &timeInfo
  );

  return String(timestamp);
}

// =====================================================
// 18. SERIAL MONITOR
// =====================================================

void printSystemStatus() {

  Serial.println();
  Serial.println("================================");
  Serial.println("       SMART TRANSPORT BOX");
  Serial.println("================================");

  Serial.println("--- TEMPERATURE ---");

  if (temperatureAvailable) {

    Serial.print("Temperature: ");
    Serial.print(temperatureC, 2);
    Serial.println(" C");

  } else {

    Serial.println("Temperature unavailable.");
  }

  Serial.println();
  Serial.println("--- MOTION SENSOR ---");

  if (mpuAvailable) {

    Serial.print("Sensor identity: 0x");
    Serial.println(mpuIdentity, HEX);

    Serial.print("Acceleration X: ");
    Serial.print(accelX, 3);
    Serial.println(" g");

    Serial.print("Acceleration Y: ");
    Serial.print(accelY, 3);
    Serial.println(" g");

    Serial.print("Acceleration Z: ");
    Serial.print(accelZ, 3);
    Serial.println(" g");

    Serial.print("Gyroscope X: ");
    Serial.print(gyroX, 2);
    Serial.println(" deg/s");

    Serial.print("Gyroscope Y: ");
    Serial.print(gyroY, 2);
    Serial.println(" deg/s");

    Serial.print("Gyroscope Z: ");
    Serial.print(gyroZ, 2);
    Serial.println(" deg/s");

    Serial.print("Vibration RMS: ");
    Serial.print(vibrationG, 4);
    Serial.println(" g");

    Serial.print("Vibration Peak: ");
    Serial.print(vibrationPeakG, 4);
    Serial.println(" g");

  } else {

    Serial.println("Motion sensor unavailable.");
  }

  Serial.println();
  Serial.println("--- CONNECTION STATUS ---");

  Serial.print("Wi-Fi: ");
  Serial.println(
    WiFi.status() == WL_CONNECTED
      ? "CONNECTED"
      : "OFFLINE"
  );

  Serial.print("MQTT: ");
  Serial.println(
    mqttClient.connected()
      ? "CONNECTED"
      : "OFFLINE"
  );

  Serial.println("================================");
}

// =====================================================
// 19. TFT DASHBOARD LAYOUT
// =====================================================

void drawDashboardLayout() {

  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(
    0, 0, TFT_WIDTH, 32,
    ST77XX_BLUE
  );

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.print("SMART BOX");

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(8, 43);
  tft.print("TEMPERATURE");

  tft.drawFastHLine(
    8, 108, 224,
    ST77XX_WHITE
  );

  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(8, 117);
  tft.print("MOTION (g)");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);

  tft.setCursor(8, 145);
  tft.print("X:");

  tft.setCursor(88, 145);
  tft.print("Y:");

  tft.setCursor(168, 145);
  tft.print("Z:");

  tft.setCursor(8, 165);
  tft.print("Vibration RMS:");

  tft.setCursor(8, 180);
  tft.print("Vibration Peak:");

  tft.drawFastHLine(
    8, 198, 224,
    ST77XX_WHITE
  );

  tft.setCursor(8, 207);
  tft.print("WiFi:");

  tft.setCursor(8, 222);
  tft.print("MQTT:");
}

// =====================================================
// 20. TFT VALUE UPDATES
// =====================================================

void updateDashboardValues() {

  // Temperature
  tft.fillRect(
    8, 67, 220, 34,
    ST77XX_BLACK
  );

  tft.setCursor(8, 67);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);

  if (temperatureAvailable) {

    tft.print(temperatureC, 1);
    tft.print(" C");

  } else {

    tft.print("--.- C");
  }

  // Acceleration
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  tft.fillRect(28, 143, 52, 15, ST77XX_BLACK);
  tft.fillRect(108, 143, 52, 15, ST77XX_BLACK);
  tft.fillRect(188, 143, 48, 15, ST77XX_BLACK);

  if (mpuAvailable) {

    tft.setCursor(28, 145);
    tft.print(accelX, 2);

    tft.setCursor(108, 145);
    tft.print(accelY, 2);

    tft.setCursor(188, 145);
    tft.print(accelZ, 2);

  } else {

    tft.setCursor(28, 145);
    tft.setTextColor(ST77XX_RED);
    tft.print("NO SENSOR");
  }

  // Vibration RMS
  tft.fillRect(112, 165, 120, 12, ST77XX_BLACK);

  tft.setCursor(112, 165);
  tft.setTextColor(ST77XX_GREEN);

  if (mpuAvailable) {

    tft.print(vibrationG, 3);
    tft.print(" g");

  } else {

    tft.print("--");
  }

  // Vibration peak
  tft.fillRect(112, 180, 120, 12, ST77XX_BLACK);

  tft.setCursor(112, 180);

  if (mpuAvailable) {

    tft.print(vibrationPeakG, 3);
    tft.print(" g");

  } else {

    tft.print("--");
  }

  // Wi-Fi
  tft.fillRect(48, 207, 175, 12, ST77XX_BLACK);
  tft.setCursor(48, 207);

  tft.setTextColor(
    WiFi.status() == WL_CONNECTED
      ? ST77XX_GREEN
      : ST77XX_RED
  );

  tft.print(
    WiFi.status() == WL_CONNECTED
      ? "CONNECTED"
      : "OFFLINE"
  );

  // MQTT
  tft.fillRect(48, 222, 175, 12, ST77XX_BLACK);
  tft.setCursor(48, 222);

  tft.setTextColor(
    mqttClient.connected()
      ? ST77XX_GREEN
      : ST77XX_RED
  );

  tft.print(
    mqttClient.connected()
      ? "CONNECTED"
      : "OFFLINE"
  );
}

// =====================================================
// 21. MQTT TELEMETRY PUBLISHING
// =====================================================

void publishTelemetry() {

  if (!mqttClient.connected()) {
    return;
  }

  if (!temperatureAvailable || !mpuAvailable) {

    Serial.println(
      "Telemetry skipped: a sensor is unavailable."
    );

    return;
  }

  String timestamp = getTimestamp();

  if (timestamp.length() == 0) {

    Serial.println(
      "Telemetry skipped: UTC time not synchronized."
    );

    return;
  }

  char payload[256];

  snprintf(
    payload,
    sizeof(payload),
    "{\"device_id\":\"%s\","
    "\"journey_id\":\"%s\","
    "\"timestamp\":\"%s\","
    "\"temperature_c\":%.2f,"
    "\"vibration_g\":%.3f}",
    DEVICE_ID,
    JOURNEY_ID,
    timestamp.c_str(),
    temperatureC,
    vibrationG
  );

  if (mqttClient.publish(MQTT_TOPIC, payload)) {

    Serial.println();
    Serial.println("MQTT MESSAGE PUBLISHED:");
    Serial.println(payload);

  } else {

    Serial.println("MQTT publish failed.");
  }
}

// =====================================================
// 22. SETUP
// =====================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println("STARTING SMART TRANSPORT BOX");
  Serial.println("================================");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("I2C initialized.");

  // TFT
  tft.init(TFT_WIDTH, TFT_HEIGHT);
  tft.setRotation(1);

  drawDashboardLayout();

  Serial.println("TFT initialized.");

  // Motion sensor
  mpuAvailable = initializeMPU();

  if (mpuAvailable) {

    // The box must remain still during this calibration.
    if (!calibrateGyroscope()) {
      mpuAvailable = false;
      Serial.println("Motion sensor disabled after calibration failure.");
    }
  }

  // DS18B20
  temperatureSensor.begin();
  temperatureSensor.setResolution(10);
  temperatureSensor.setWaitForConversion(false);

  Serial.print("DS18B20 devices detected: ");
  Serial.println(temperatureSensor.getDeviceCount());

  startTemperatureConversion();

  // MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(256);

  // Wi-Fi
  connectWiFi();

  Serial.println();
  Serial.println("System initialization completed.");
}

// =====================================================
// 23. MAIN LOOP
// =====================================================

void loop() {

  unsigned long now = millis();

  // Wi-Fi management
  connectWiFi();

  // MQTT management
  if (WiFi.status() == WL_CONNECTED) {

    connectMQTT();
    mqttClient.loop();
  }

  // Retry motion sensor initialization if unavailable
  if (!mpuAvailable &&
      now - lastMpuAttempt >= MPU_RETRY_INTERVAL) {

    lastMpuAttempt = now;

    mpuAvailable = initializeMPU();

    if (mpuAvailable) {

      if (!calibrateGyroscope()) {
        mpuAvailable = false;
      }
    }
  }

  // Motion sampling
  if (mpuAvailable &&
      now - lastMotionSample >= MOTION_INTERVAL) {

    lastMotionSample = now;

    readMotion();
  }

  // Vibration calculation
  calculateVibration();

  // Temperature conversion
  updateTemperature();

  // Serial status
  if (now - lastSensorRead >= SENSOR_INTERVAL) {

    lastSensorRead = now;

    printSystemStatus();
  }

  // TFT refresh
  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {

    lastDisplayUpdate = now;

    updateDashboardValues();
  }

  // MQTT telemetry
  if (now - lastMqttPublish >= MQTT_INTERVAL) {

    lastMqttPublish = now;

    publishTelemetry();
  }
}
