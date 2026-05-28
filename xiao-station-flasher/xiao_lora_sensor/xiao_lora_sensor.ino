/*
 * XIAO ESP32S3 + Wio-SX1262 + SHT35 + soil moisture
 * Raw LoRa uplink to a local WM1302 gateway (no LoRaWAN, no TTN, no internet).
 *
 * The WM1302 gateway is configured with `"lorawan_public": 1`, so its SX1302
 * demodulator only locks onto packets with sync word 0x34 ("public" / LoRaWAN-
 * compatible). RadioLib's default sync word is 0x12 ("private") and would
 * be ignored — hence the explicit `radio.setSyncWord(0x34)` below.
 *
 * Identity
 *   Each station has a stable, globally-unique physical identity derived from
 *   the ESP32-S3 factory MAC (read from eFuse). A 16-bit "short ID" is derived
 *   from the 6-byte MAC via FNV-1a-16 (FNV-1a folded to 16 bits) and that is
 *   what travels in the LoRa payload — the full MAC stays on-device but is
 *   reported on the serial banner at boot so the flashing script can record it.
 *
 *   Manual override: if -DSTATION_ID_OVERRIDE=N is passed at compile time, the
 *   short ID becomes N instead of the MAC-derived hash. Useful for testing and
 *   forcing a known ID. The MAC is still read and printed regardless. The
 *   flash_xiao_station.sh script uses MAC mode by default; pass --force-id N
 *   to take the override path.
 *
 * Required Arduino libraries (Library Manager)
 *   - RadioLib by Jan Gromes, version 6.0.0 or newer (any modern version works)
 *   - Grove - Temperature & Humidity Sensor (SHT3x) by Seeed Studio
 *
 * Board package
 *   - "esp32" by Espressif Systems, 3.x (ESP-IDF 5.x based)
 *   - Board selected: "XIAO_ESP32S3"
 */

#include <Wire.h>
#include "Seeed_SHT35.h"
#include <RadioLib.h>
#include "esp_mac.h"

// ============================================================================
// === PER-DEVICE CONFIGURATION ==============================================
// ============================================================================

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
// === IDENTITY ==============================================================
// ============================================================================

uint8_t  STATION_MAC[6] = {0};
uint16_t STATION_SHORT_ID = 0;

// FNV-1a hash, 32-bit pipeline folded to 16 bits via XOR of the halves.
// Deterministic, no library, good distribution for a 6-byte input.
static uint16_t fnv1a_16(const uint8_t *data, size_t len) {
  const uint32_t FNV_PRIME  = 0x01000193UL;
  const uint32_t FNV_OFFSET = 0x811C9DC5UL;
  uint32_t h = FNV_OFFSET;
  for (size_t i = 0; i < len; i++) {
    h ^= (uint32_t)data[i];
    h *= FNV_PRIME;
  }
  return (uint16_t)((h >> 16) ^ (h & 0xFFFFUL));
}

// ============================================================================

bool radioReady = false;

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println();
  Serial.println(F("=== XIAO sensor station ==="));

  // --- Identity: read factory MAC from eFuse, derive 16-bit short ID --------
  esp_efuse_mac_get_default(STATION_MAC);

  #ifdef STATION_ID_OVERRIDE
    STATION_SHORT_ID = (uint16_t)(STATION_ID_OVERRIDE);
  #else
    STATION_SHORT_ID = fnv1a_16(STATION_MAC, 6);
  #endif

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           STATION_MAC[0], STATION_MAC[1], STATION_MAC[2],
           STATION_MAC[3], STATION_MAC[4], STATION_MAC[5]);
  Serial.print(F("STATION MAC: "));      Serial.println(macStr);
  Serial.print(F("STATION SHORT_ID: 0x"));
  if (STATION_SHORT_ID < 0x1000) Serial.print('0');
  if (STATION_SHORT_ID < 0x0100) Serial.print('0');
  if (STATION_SHORT_ID < 0x0010) Serial.print('0');
  Serial.println(STATION_SHORT_ID, HEX);
  #ifdef STATION_ID_OVERRIDE
    Serial.println(F("(short ID is from STATION_ID_OVERRIDE, not MAC hash)"));
  #endif

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
  // === Pack 7-byte payload ==================================================
  // ==========================================================================
  //   [0..1] uint16  short station ID BE   (MAC-derived, or STATION_ID_OVERRIDE)
  //   [2..3] int16   temperature × 100 BE  (0x8000 sentinel = SHT35 fail)
  //   [4..5] uint16  humidity × 100 BE     (0xFFFF sentinel = SHT35 fail)
  //   [6]    uint8   soil moisture %       (0..100)

  int16_t  tEnc = shtOk ? (int16_t)(temp * 100.0f) : (int16_t)0x8000;
  uint16_t hEnc = shtOk ? (uint16_t)(hum  * 100.0f) : 0xFFFF;
  uint8_t  sEnc = (uint8_t)constrain((int)soilPct, 0, 100);

  uint8_t payload[7];
  payload[0] = (STATION_SHORT_ID >> 8) & 0xFF;
  payload[1] =  STATION_SHORT_ID       & 0xFF;
  payload[2] = (tEnc >> 8) & 0xFF;
  payload[3] =  tEnc       & 0xFF;
  payload[4] = (hEnc >> 8) & 0xFF;
  payload[5] =  hEnc       & 0xFF;
  payload[6] = sEnc;

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
