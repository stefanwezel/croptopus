/*
 * XIAO ESP32S3 + Wio-SX1262 + SHT35 + soil moisture
 * Raw LoRa uplink to a local WM1302 gateway (no LoRaWAN, no TTN, no internet).
 *
 * The WM1302 gateway is configured with `"lorawan_public": 1`, so its SX1302
 * demodulator only locks onto packets with sync word 0x34 ("public" / LoRaWAN-
 * compatible). RadioLib's default sync word is 0x12 ("private") and would
 * be ignored — hence the explicit `radio.setSyncWord(0x34)` below.
 *
 * Each device must have a unique STATION_ID so the BBB listener can tell them
 * apart. Don't use 0.
 *
 * Required Arduino libraries (Library Manager)
 *   - RadioLib by Jan Gromes, version 6.0.0 or newer (any modern version works)
 *   - Grove - Temperature & Humidity Sensor (SHT3x) by Seeed Studio
 *
 * Board package
 *   - "esp32" by Espressif Systems, 2.0.14 or newer
 *   - Board selected: "XIAO_ESP32S3"
 *
 * Flashing
 *   - Manual: open in Arduino IDE, edit STATION_ID default below if needed, upload.
 *   - Scripted: use flash_xiao_station.sh which passes STATION_ID via -DSTATION_ID_OVERRIDE=N
 *     and leaves this source file untouched.
 */

#include <Wire.h>
#include "Seeed_SHT35.h"
#include <RadioLib.h>

// ============================================================================
// === PER-DEVICE CONFIGURATION ==============================================
// ============================================================================

// Station identifier (1-255). UNIQUE per device. Reserve 0.
//
// At compile time, STATION_ID_OVERRIDE (passed as a -D flag) takes precedence
// over the default below. The flash_xiao_station.sh script always sets it.
// If you flash from Arduino IDE without the script, the default is used —
// edit it here if you need a different value.
#ifdef STATION_ID_OVERRIDE
  const uint8_t STATION_ID = STATION_ID_OVERRIDE;
#else
  const uint8_t STATION_ID = 1;
#endif

// Uplink interval in milliseconds.
//   60000  (1 min)  — fine for raw LoRa; you set your own rules here.
//   300000 (5 min)  — sensible for a sensor running long-term.
const unsigned long UPLINK_INTERVAL_MS = 60UL * 1000UL;

// Soil moisture ADC calibration. Tune to your sensor + soil.
const int DRY_VALUE = 2500;
const int WET_VALUE = 1200;

// ============================================================================
// === LORA RF PARAMETERS ====================================================
// ============================================================================
//
// Frequency must be one of the 8 channels the WM1302 monitors for EU868:
//   867.1, 867.3, 867.5, 867.7, 867.9, 868.1, 868.3, 868.5 MHz
//
// All stations should share the same SF/BW/CR so the gateway demodulates them
// the same way. SF7+125kHz is fastest and uses the least airtime.
//
// Sync word 0x34 = "public network" — matches the WM1302's configuration.

const float   LORA_FREQ_MHZ = 868.1f;
const float   LORA_BW_KHZ   = 125.0f;
const uint8_t LORA_SF       = 7;
const uint8_t LORA_CR       = 5;       // coding rate 4/5
const int8_t  LORA_TX_DBM   = 14;
const uint8_t LORA_SYNC     = 0x34;    // public sync word

// ============================================================================
// === HARDWARE PIN MAPPING ==================================================
// ============================================================================

#define SDAPIN SDA
#define SCLPIN SCL
const int SOIL_PIN = A0;

SHT35 sensor(SCLPIN);

// Wio-SX1262 carrier: NSS=41, DIO1=39, RST=42, BUSY=40
SX1262 radio = new Module(41, 39, 42, 40);

// ============================================================================

bool radioReady = false;

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.print(F("=== XIAO sensor station #"));
  Serial.print(STATION_ID);
  Serial.println(F(" ==="));

  // --- I2C + SHT35 ----------------------------------------------------------
  Wire.begin(SDAPIN, SCLPIN);
  if (sensor.init()) {
    Serial.println(F("SHT35 init FAILED — temperature/humidity will be NaN"));
  } else {
    Serial.println(F("SHT35 init OK"));
  }
  pinMode(SOIL_PIN, INPUT);

  // --- SX1262 radio init ----------------------------------------------------
  Serial.print(F("Radio init... "));  
  // RadioLib 7.7.0 begin() structure:
  // freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage
  // Setting tcxoVoltage to 0.0f disables TCXO checking and switches to standard XTAL mode.
  int16_t state = radio.begin(LORA_FREQ_MHZ);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("FAILED, code ")); Serial.println(state);
    Serial.println(F("Sensor reads will continue without RF transmission."));
    return;
  }
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(LORA_SF);
  radio.setCodingRate(LORA_CR);
  radio.setOutputPower(LORA_TX_DBM);
  radio.setSyncWord(LORA_SYNC);
  Serial.println(F("OK"));

  Serial.print(F("RF: "));        Serial.print(LORA_FREQ_MHZ);
  Serial.print(F(" MHz, SF"));    Serial.print(LORA_SF);
  Serial.print(F(", BW "));       Serial.print(LORA_BW_KHZ);
  Serial.print(F(" kHz, CR 4/")); Serial.print(LORA_CR);
  Serial.print(F(", "));          Serial.print(LORA_TX_DBM);
  Serial.println(F(" dBm"));

  radioReady = true;
}

void loop() {
  // ==========================================================================
  // === SENSOR READS — always run, regardless of radio state ================
  // ==========================================================================

  float temp = 0.0f, hum = 0.0f;
  bool shtOk = (NO_ERROR ==
                sensor.read_meas_data_single_shot(HIGH_REP_WITH_STRCH, &temp, &hum));

  if (shtOk) {
    Serial.print(F("T = "));  Serial.print(temp, 2); Serial.print(F(" °C   "));
    Serial.print(F("RH = ")); Serial.print(hum,  2); Serial.println(F(" %"));
  } else {
    Serial.println(F("SHT35 read FAILED"));
  }

  int rawSoil = analogRead(SOIL_PIN);
  float soilV = rawSoil * (3.3f / 4095.0f);
  float soilPct;
  if (rawSoil <= WET_VALUE)      soilPct = 100.0f;
  else if (rawSoil >= DRY_VALUE) soilPct =   0.0f;
  else                            soilPct = 100.0f * (DRY_VALUE - rawSoil) /
                                            (float)(DRY_VALUE - WET_VALUE);

  Serial.print(F("Soil ADC = ")); Serial.print(rawSoil);
  Serial.print(F(" ("));          Serial.print(soilV, 2);
  Serial.print(F(" V) → "));      Serial.print(soilPct, 1);
  Serial.println(F(" %"));

  // ==========================================================================
  // === Pack 6-byte payload ==================================================
  // ==========================================================================
  //   [0]    uint8   station ID
  //   [1..2] int16   temperature × 100 BE  (0x8000 sentinel = SHT35 fail)
  //   [3..4] uint16  humidity × 100 BE     (0xFFFF sentinel = SHT35 fail)
  //   [5]    uint8   soil moisture %       (0..100)

  int16_t  tEnc = shtOk ? (int16_t)(temp * 100.0f) : (int16_t)0x8000;
  uint16_t hEnc = shtOk ? (uint16_t)(hum  * 100.0f) : 0xFFFF;
  uint8_t  sEnc = (uint8_t)constrain((int)soilPct, 0, 100);

  uint8_t payload[6];
  payload[0] = STATION_ID;
  payload[1] = (tEnc >> 8) & 0xFF;
  payload[2] =  tEnc       & 0xFF;
  payload[3] = (hEnc >> 8) & 0xFF;
  payload[4] =  hEnc       & 0xFF;
  payload[5] = sEnc;

  // --- Transmit ------------------------------------------------------------
  if (radioReady) {
    Serial.print(F("TX: "));
    for (size_t i = 0; i < sizeof(payload); i++) {
      if (payload[i] < 0x10) Serial.print('0');
      Serial.print(payload[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    int16_t tx = radio.transmit(payload, sizeof(payload));
    if (tx == RADIOLIB_ERR_NONE) {
      Serial.println(F("TX OK"));
    } else {
      Serial.print(F("TX FAILED, code ")); Serial.println(tx);
    }
  } else {
    Serial.println(F("(radio not ready, skipping TX)"));
  }

  Serial.println(F("---------------"));
  delay(UPLINK_INTERVAL_MS);
}
