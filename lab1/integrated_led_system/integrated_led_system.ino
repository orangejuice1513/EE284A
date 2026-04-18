/* Lab 1 Integrated LED system
 * Reads BME688 (SPI) and analog tmp36 in sync.
 * Applies EMA smoothing to both, lights LED when they disagree by >= THRESHOLD.
 */

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define LED          32
#define TMP_PIN      A2
#define BME_SCK       5
#define BME_MISO     21
#define BME_MOSI     19
#define BME_CS       14

#define THRESHOLD    2.5f   // temp difference that triggers LED
#define ALPHA        (2.0f / (50 + 1))  // EMA with N=50 equivalent window
#define SAMPLE_MS    100    // delay between samples

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

float emaBME    = 0;
float emaAnalog = 0;
bool  emaInit   = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // analog sensor config
  analogReadResolution(12);

  // BME688 config
  if (!bme.begin()) {
    Serial.println("BME688 not found!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_NONE);
  bme.setPressureOversampling(BME680_OS_NONE);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);

  Serial.println("Ready. Sampling both sensors...");
}

void loop() {
  // Read both sensors at the same timestamp
  if (!bme.performReading()) return;
  float bmeTemp    = bme.temperature;
  float analogTemp = readAnalogTempC();


  // update ema 
  if (!emaInit) { // initialize first reading so we don't start from 0
    emaBME    = bmeTemp;
    emaAnalog = analogTemp;
    emaInit   = true;
  } else {
    emaBME    = ALPHA * bmeTemp    + (1.0f - ALPHA) * emaBME;
    emaAnalog = ALPHA * analogTemp + (1.0f - ALPHA) * emaAnalog;
  }

  // compare filtered values
  float diff = fabs(emaBME - emaAnalog);
  digitalWrite(LED, diff >= THRESHOLD ? HIGH : LOW);

  Serial.printf("BME: %.2f°C | Analog: %.2f°C | Diff: %.3f°C | LED: %s\n",
                emaBME, emaAnalog, diff,
                diff >= THRESHOLD ? "ON" : "OFF");

  delay(SAMPLE_MS);
}


float readAnalogTempC() {
  uint32_t mV = analogReadMilliVolts(TMP_PIN);  // calibrated reading in millivolts
  float voltage = mV / 1000.0f;                 // convert to volts
  return (voltage - 0.5f) * 100.0f;             // TMP36-style: 10mV/°C, 500mV offset at 0°C
}


