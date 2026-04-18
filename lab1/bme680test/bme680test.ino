#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "esp_task_wdt.h"

#define BME_CS   14
#define BME_MOSI 19
#define BME_MISO 21
#define BME_SCK   5

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

#define TOTAL_SAMPLES 5000
#define N_SMALL       50
#define N_LARGE       500
const float ALPHA_SMALL = 2.0f / (N_SMALL + 1);
const float ALPHA_LARGE = 2.0f / (N_LARGE + 1);

// Ring buffers
float bufSmall[N_SMALL], bufLarge[N_LARGE];
float maBufS[N_SMALL],   maBufL[N_LARGE];
float sortBuf[N_LARGE];

int   idxS=0, idxL=0, cntS=0, cntL=0;
int   maIdxS=0, maIdxL=0, maCntS=0, maCntL=0;
float maSumS=0, maSumL=0;
float emaS=0, emaL=0;
bool  emaInitS=false, emaInitL=false;

#define NF 7
float  statMin[NF], statMax[NF];
double statSum[NF], statSumSq[NF];
int    statCnt[NF];

void statUpdate(int f, float v) {
  if (v < statMin[f]) statMin[f] = v;
  if (v > statMax[f]) statMax[f] = v;
  statSum[f]   += v;
  statSumSq[f] += (double)v * v;
  statCnt[f]++;
}

float getMedian(float* src, int n) {
  memcpy(sortBuf, src, n * sizeof(float));
  for (int i = 1; i < n; i++) {
    float k = sortBuf[i]; int j = i-1;
    while (j >= 0 && sortBuf[j] > k) { sortBuf[j+1] = sortBuf[j]; j--; }
    sortBuf[j+1] = k;
  }
  return (n % 2) ? sortBuf[n/2] : (sortBuf[n/2-1] + sortBuf[n/2]) * 0.5f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int f = 0; f < NF; f++) {
    statMin[f]=1e9; statMax[f]=-1e9;
    statSum[f]=0;   statSumSq[f]=0; statCnt[f]=0;
  }
  // Zero out ring buffers so subtraction on first pass is safe
  memset(maBufS, 0, sizeof(maBufS));
  memset(maBufL, 0, sizeof(maBufL));

  if (!bme.begin()) {
    Serial.println("BME680 not found!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_NONE);
  bme.setPressureOversampling(BME680_OS_NONE);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(0, 0);

  Serial.println("Starting 5000 samples...");

  for (int i = 0; i < TOTAL_SAMPLES; i++) {

    esp_task_wdt_reset(); // keep watchdog happy

    if (!bme.performReading()) { i--; continue; }
    float raw = bme.temperature;

    // 0: Raw
    statUpdate(0, raw);

    // 1: MA-50
    maSumS -= maBufS[maIdxS];
    maBufS[maIdxS] = raw; maSumS += raw;
    maIdxS = (maIdxS+1) % N_SMALL;
    if (maCntS < N_SMALL) maCntS++;
    statUpdate(1, maSumS / maCntS);

    // 2: MA-500
    maSumL -= maBufL[maIdxL];
    maBufL[maIdxL] = raw; maSumL += raw;
    maIdxL = (maIdxL+1) % N_LARGE;
    if (maCntL < N_LARGE) maCntL++;
    statUpdate(2, maSumL / maCntL);

    // 3: Median-50
    bufSmall[idxS] = raw;
    idxS = (idxS+1) % N_SMALL;
    if (cntS < N_SMALL) cntS++;
    statUpdate(3, getMedian(bufSmall, cntS));

    // 4: Median-500 — only sort when buffer is full to avoid slow warmup
    bufLarge[idxL] = raw;
    idxL = (idxL+1) % N_LARGE;
    if (cntL < N_LARGE) cntL++;
    if (cntL == N_LARGE) statUpdate(4, getMedian(bufLarge, N_LARGE));

    // 5: EMA alpha=2/(50+1)
    if (!emaInitS) { emaS = raw; emaInitS = true; }
    else emaS = ALPHA_SMALL * raw + (1.0f - ALPHA_SMALL) * emaS;
    statUpdate(5, emaS);

    // 6: EMA alpha=2/(500+1)
    if (!emaInitL) { emaL = raw; emaInitL = true; }
    else emaL = ALPHA_LARGE * raw + (1.0f - ALPHA_LARGE) * emaL;
    statUpdate(6, emaL);

    if ((i+1) % 500 == 0)
      Serial.printf("  %d / %d\n", i+1, TOTAL_SAMPLES);
  }

  const char* names[NF] = {
    "Raw","MA-50","MA-500","Median-50","Median-500","EMA-50","EMA-500"
  };
  Serial.println("\nFilter        |   Min   |   Max   |  Mean   | StdDev");
  Serial.println("--------------+---------+---------+---------+--------");
  for (int f = 0; f < NF; f++) {
    double m  = statSum[f] / statCnt[f];
    float  sd = sqrt(statSumSq[f]/statCnt[f] - m*m);
    Serial.printf("%-13s | %7.3f | %7.3f | %7.3f | %7.5f\n",
                  names[f], statMin[f], statMax[f], (float)m, sd);
  }
  Serial.println("\nDone.");
}

void loop() {}