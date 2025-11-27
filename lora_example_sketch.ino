#include <Wire.h>
#include "Seeed_SHT35.h"
#include <RadioLib.h>

// --- I2C pins (from your original sketch) ---
#define SDAPIN SDA
#define SCLPIN SCL

#define SERIAL Serial

// Soil moisture sensor: Grove analog port wired to A0 on XIAO
const int SOIL_PIN = A0;

// SHT35 object on SCL pin
SHT35 sensor(SCLPIN);

// --- LoRa: SX1262 on Wio-SX1262 for XIAO ---
// Pin mapping from Seeed's Wio-SX1262 + XIAO ESP32S3 kit examples:
// NSS  = GPIO41
// DIO1 = GPIO39
// RST  = GPIO42
// BUSY = GPIO40
SX1262 radio = new Module(41, 39, 42, 40);

// LoRa configuration (EU868-friendly defaults)
const float   LORA_FREQ_MHZ       = 868.1f;   // adjust if needed for your region
const float   LORA_BW_KHZ         = 125.0f;   // 125 kHz bandwidth
const uint8_t LORA_SF             = 7;        // spreading factor
const uint8_t LORA_CR             = 5;        // coding rate = 4/5
const int8_t  LORA_TX_POWER_DBM   = 14;       // conservative, within SX1262 spec

bool loraReady = false;  // only transmit if init succeeded

void setup() {
  // --- Serial for debug ---
  SERIAL.begin(115200);
  delay(200);
  SERIAL.println();
  SERIAL.println("Serial start (XIAO ESP32S3 with LoRa)...");

  // --- I2C + SHT35 init ---
  Wire.begin(SDAPIN, SCLPIN);

  if (sensor.init()) {
    SERIAL.println("SHT35 sensor init FAILED!!!");
  } else {
    SERIAL.println("SHT35 sensor init OK.");
  }

  // Soil moisture input pin
  pinMode(SOIL_PIN, INPUT);

  // --- LoRa / SX1262 init ---
  SERIAL.println("Initializing SX1262 LoRa radio...");
  int16_t state = radio.begin();  // use RadioLib defaults for LoRa mode

  if (state != RADIOLIB_ERR_NONE) {
    SERIAL.print("LoRa init failed, code ");
    SERIAL.println(state);
    SERIAL.println("Continuing without LoRa transmission.");
    loraReady = false;
  } else {
    SERIAL.println("LoRa init OK.");

    // Configure basic LoRa parameters (all safe operations)
    radio.setFrequency(LORA_FREQ_MHZ);
    radio.setSpreadingFactor(LORA_SF);
    radio.setBandwidth(LORA_BW_KHZ);
    radio.setCodingRate(LORA_CR);
    radio.setOutputPower(LORA_TX_POWER_DBM);

    SERIAL.print("LoRa params: ");
    SERIAL.print(LORA_FREQ_MHZ);
    SERIAL.print(" MHz, SF");
    SERIAL.print(LORA_SF);
    SERIAL.print(", BW ");
    SERIAL.print(LORA_BW_KHZ);
    SERIAL.print(" kHz, CR 4/");
    SERIAL.print(LORA_CR);
    SERIAL.print(", Pout ");
    SERIAL.print(LORA_TX_POWER_DBM);
    SERIAL.println(" dBm");

    loraReady = true;
  }
}

void loop() {
  // --- SHT35 temperature & humidity ---
  float temp = 0.0f;
  float hum  = 0.0f;

  bool shtOk = true;
  if (NO_ERROR != sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum)) {
    SERIAL.println("SHT35 read FAILED!!");
    shtOk = false;
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

  // Crude “percentage” estimate, tune for your setup
  const int DRY_VALUE = 2500;  // adjust based on calibration
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

  // --- LoRa transmit ---
  if (loraReady) {
    String payload;
    payload.reserve(64);

    payload += "T=";
    if (shtOk) {
      payload += String(temp, 2);
    } else {
      payload += "NaN";
    }
    payload += "C ";

    payload += "H=";
    if (shtOk) {
      payload += String(hum, 2);
    } else {
      payload += "NaN";
    }
    payload += "% ";

    payload += "SM=";
    payload += String(soilPercent, 1);
    payload += "%";

    SERIAL.print("LoRa TX -> ");
    SERIAL.println(payload);

    int16_t txState = radio.transmit(payload.c_str());
    if (txState == RADIOLIB_ERR_NONE) {
      SERIAL.println("LoRa transmit OK.");
    } else {
      SERIAL.print("LoRa transmit FAILED, code ");
      SERIAL.println(txState);
    }
  } else {
    SERIAL.println("LoRa not initialized, skipping transmit.");
  }

  SERIAL.println("---------------");
  delay(2000);  // 2s interval; TX duty cycle is very low and safe
}
