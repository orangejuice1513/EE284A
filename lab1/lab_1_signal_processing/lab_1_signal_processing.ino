#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define BME_SCK  5
#define BME_MISO 21
#define BME_MOSI 19
#define BME_CS   14
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); //use SPI 

#define TOTAL_SAMPLES 5000

class MovingAverage {
// keeps a ring buffer of the last N samples and a running sum 
public:
  void init(float* storage, int n) {
    buf  = storage;
    size = n;
    idx  = 0;
    cnt  = 0;
    sum  = 0;
    memset(buf, 0, n * sizeof(float));
  }

  float update(float val) {
    sum -= buf[idx];       // remove the value about to be overwritten
    buf[idx] = val;        // write new value
    sum += val;            // add it to running sum
    idx = (idx + 1) % size;
    if (cnt < size) cnt++;
    return sum / cnt;
  }

private:
  float* buf;
  int    size, idx, cnt;
  float  sum;
};

float sortScratch[500];  // shared scratch space, sized for the largest N

class MovingMedian {
// keeps a ring buffer and sorts a copy every update to find the middle value.
// Returns NAN until buffer is full, so warmup zeros don't pollute stats.
public:
  void init(float* storage, int n) {
    buf  = storage;
    size = n;
    idx  = 0;
    cnt  = 0;
    memset(buf, 0, n * sizeof(float));
  }

  float update(float val) {
    buf[idx] = val;
    idx = (idx + 1) % size;
    if (cnt < size) cnt++;
    if (cnt < size) return NAN;  // wait until buffer is full

    // insertion sort for median 
    memcpy(sortScratch, buf, size * sizeof(float));
    for (int i = 1; i < size; i++) {
      float key = sortScratch[i];
      int j = i - 1;
      while (j >= 0 && sortScratch[j] > key) {
        sortScratch[j + 1] = sortScratch[j];
        j--;
      }
      sortScratch[j + 1] = key;
    }
    return (size % 2) ? sortScratch[size / 2]
                      : (sortScratch[size/2 - 1] + sortScratch[size/2]) * 0.5f;
  }

private:
  float* buf;
  int    size, idx, cnt;
};

class EMA { //exponential moving average 
public:
  void init(float a) {
    alpha   = a;
    value   = 0;
    initted = false;
  }

  float update(float val) {
    if (!initted) { value = val; initted = true; }
    else value = alpha * val + (1.0f - alpha) * value;
    return value;
  }

private:
  float alpha, value;
  bool  initted;
};

struct Stats {
  float  minVal =  999.0;
  float  maxVal = -999.0;
  double sum    = 0;
  double sumSq  = 0;
  int    count  = 0;

  void update(float val) {
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
    sum   += val;
    sumSq += (double)val * val;
    count++;
  }
  float mean()   { return sum / count; }
  float stddev() { return sqrt(sumSq / count - (double)mean() * mean()); }
};

float maBuf50[50],   maBuf500[500];
float medBuf50[50],  medBuf500[500];

MovingAverage ma50,  ma500;
MovingMedian  med50, med500;
EMA           ema50, ema500;

Stats statsRaw, statsMA50, statsMA500, statsEMA50, statsEMA500, statsMed50, statsMed500;

void printRow(const char* label, Stats& s) {
  Serial.printf("%-12s | %7.4f | %7.4f | %7.4f | %9.6f\n",
                label, s.minVal, s.maxVal, s.mean(), s.stddev());
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // initialize all filters with their N values
  ma50.init(maBuf50, 50);
  ma500.init(maBuf500, 500);
  med50.init(medBuf50, 50);
  med500.init(medBuf500, 500);
  ema50.init(2.0f / (50 + 1));
  ema500.init(2.0f / (500 + 1));

  //iInitialize sensor
  if (!bme.begin()) {
    Serial.println("BME688 not found!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_NONE);
  bme.setPressureOversampling(BME680_OS_NONE);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);

  Serial.println("BME688 ready. Starting experiment...\n");

  int n = 0;
  while (n < TOTAL_SAMPLES) {

    if (!bme.performReading()) continue;
    float raw = bme.temperature;

    statsRaw.update(raw);
    statsMA50.update(ma50.update(raw));
    statsMA500.update(ma500.update(raw));
    statsEMA50.update(ema50.update(raw));
    statsEMA500.update(ema500.update(raw));

    float m50  = med50.update(raw);
    float m500 = med500.update(raw);
    if (!isnan(m50))  statsMed50.update(m50);
    if (!isnan(m500)) statsMed500.update(m500);

    n++;
  }

  // print everything 
  Serial.println("\n========= RESULTS =========");
  Serial.println("Filter       |   Min   |   Max   |  Mean   |  StdDev");
  Serial.println("-------------+---------+---------+---------+---------");
  printRow("Raw",        statsRaw);
  printRow("MA-50",      statsMA50);
  printRow("MA-500",     statsMA500);
  printRow("EMA-50",     statsEMA50);
  printRow("EMA-500",    statsEMA500);
  printRow("Median-50",  statsMed50);
  printRow("Median-500", statsMed500);
}

void loop() {}