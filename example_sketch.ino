#include <Wire.h>
#include "Seeed_SHT35.h"

// Use the board's default I2C pins.
// On XIAO ESP32S3 with Grove Base/Expansion, SDA/SCL go to the Grove I2C ports.
#define SDAPIN SDA
#define SCLPIN SCL

#define SERIAL Serial

// Soil moisture sensor is on Grove port 1 → analog pin A0
const int SOIL_PIN = A0;

// SHT35 object, using SCL pin as in Seeed example
SHT35 sensor(SCLPIN);

void setup() {
  SERIAL.begin(115200);
  delay(10);
  SERIAL.println("Serial start (XIAO ESP32S3)...");

  // Explicit I2C init (safe and matches Grove I2C routing)
  Wire.begin(SDAPIN, SCLPIN);

  // Initialize SHT35
  if (sensor.init()) {
    SERIAL.println("SHT35 sensor init failed!!!");
  } else {
    SERIAL.println("SHT35 sensor init OK.");
  }

  // Soil moisture input
  pinMode(SOIL_PIN, INPUT);
}

void loop() {
  // --- SHT35 temperature & humidity ---
  float temp = 0.0f;
  float hum  = 0.0f;

  if (NO_ERROR != sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum)) {
    SERIAL.println("SHT35 read failed!!");
  } else {
    SERIAL.println("Sensor readings:");
    SERIAL.print("temperature = ");
    SERIAL.println(temp);
    SERIAL.print("humidity = ");
    SERIAL.println(hum);
  }

  // --- Soil moisture (analog) ---
  int rawSoil = analogRead(SOIL_PIN);
  float soilVoltage = rawSoil * (3.3f / 4095.0f);  // 12-bit ADC, 0..4095

  // Crude “percentage” estimate, adjust to taste
  const int DRY_VALUE = 2500;  // tune these for your soil + sensor
  const int WET_VALUE = 1200;

  float soilPercent = 0.0f;
  if (rawSoil <= WET_VALUE) {
    soilPercent = 100.0f;
  } else if (rawSoil >= DRY_VALUE) {
    soilPercent = 0.0f;
  } else {
    soilPercent = 100.0f * (DRY_VALUE - rawSoil) / (float)(DRY_VALUE - WET_VALUE);
  }

  SERIAL.print("Soil ADC: ");
  SERIAL.print(rawSoil);
  SERIAL.print("  (");
  SERIAL.print(soilVoltage, 2);
  SERIAL.println(" V)");

  SERIAL.print("Soil moisture (rough): ");
  SERIAL.print(soilPercent, 1);
  SERIAL.println(" %");

  SERIAL.println("---------------");
  delay(2000);
}
