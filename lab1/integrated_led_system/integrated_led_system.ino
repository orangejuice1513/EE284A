/* Lab 1 Integrated LED system
 * Reads BME688 (SPI) and analog temp sensor (A2) in sync.
 * Applies EMA smoothing to both, lights LED when they disagree by >= THRESHOLD.
 */

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define LED          15
#define ADC_PIN      A2
#define BME_SCK       5
#define BME_MISO     21
#define BME_MOSI     19
#define BME_CS       14

#define THRESHOLD    1.0f   // °C difference that triggers LED
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
  analogSetPinAttenuation(ADC_PIN, ADC_6db);

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
  float analogTemp = digital_to_celsius(analogRead(ADC_PIN));

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

float digital_to_celsius(int DN) {
  // convert ADC reading to °C
  // ADC_6db attenuation on ESP32 = ~2.1V full scale at 12-bit resolution
  // sensor output: 0.5V offset + 10mV/°C (TMP36-style)
  float voltage = (2.1f / 4095.0f) * DN;
  return (voltage - 0.5f) * 100.0f;
}