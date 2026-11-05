#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// ---- Config --------------------------------------------------------------
static const int PIN_SDA = 4;        // safe pins (NOT GPIO8/9 strapping pins)
static const int PIN_SCL = 5;
static const int IMU_HZ  = 200;
static const int RING_N  = 256;      // 1.28 s of history @ 200 Hz
static const int DRAIN_N = 64;

// ---- Sample + lock-free SPSC ring buffer ---------------------------------
struct ImuSample {
  int64_t t_us;                      // monotonic, µs since boot
  float ax, ay, az;                  // m/s^2
  float gx, gy, gz;                  // rad/s
};

static ImuSample ring[RING_N];
static std::atomic<uint32_t> head{0}, tail{0};
static std::atomic<uint32_t> drops{0};

// Producer: only touches head. Drops newest on overflow, counts it.
static inline void push(const ImuSample& s) {
  uint32_t h  = head.load(std::memory_order_relaxed);
  uint32_t nx = (h + 1) % RING_N;
  if (nx != tail.load(std::memory_order_acquire)) {
    ring[h] = s;
    head.store(nx, std::memory_order_release);
  } else {
    drops.fetch_add(1, std::memory_order_relaxed);
  }
}

// Consumer: copy every sample with t_us > since_us. Returns count copied.
// Also writes the sample immediately *before* since_us (the interpolation
// anchor) into *anchor, if you pass a non-null pointer.
static int drainSince(int64_t since_us, ImuSample* out, int max, ImuSample* anchor) {
  int n = 0;
  uint32_t t = tail.load(std::memory_order_relaxed);
  uint32_t h = head.load(std::memory_order_acquire);

  if (anchor) {
    uint32_t prev = (t == 0) ? (RING_N - 1) : (t - 1);
    if (t != h) *anchor = ring[prev];       // valid: something is buffered
  }
  while (t != h && n < max) {
    if (ring[t].t_us > since_us) out[n++] = ring[t];
    t = (t + 1) % RING_N;
  }
  tail.store(t, std::memory_order_release);
  return n;
}

// ---- Sensor + tasks ------------------------------------------------------
Adafruit_MPU6050 mpu;

static void imuTask(void*) {
  TickType_t last       = xTaskGetTickCount();
  const TickType_t per  = pdMS_TO_TICKS(1000 / IMU_HZ);
  for (;;) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);            // void return in Adafruit lib

    ImuSample s;
    s.t_us = esp_timer_get_time();          // timestamp right after the read
    s.ax = a.acceleration.x;  s.ay = a.acceleration.y;  s.az = a.acceleration.z;
    s.gx = g.gyro.x;          s.gy = g.gyro.y;          s.gz = g.gyro.z;
    push(s);

    vTaskDelayUntil(&last, per);
  }
}

// Placeholder "frame arrived" task. On the C3 there is no camera, so we just
// flush every 50 ms. On a camera board: use the frame's capture timestamp as
// `now` and hand `batch` (plus `anchor`) to the fusion step.
static void frameTask(void*) {
  static ImuSample batch[DRAIN_N];
  ImuSample anchor;
  TickType_t last = xTaskGetTickCount();
  int64_t last_frame_us = 0;

  for (;;) {
    int64_t now = esp_timer_get_time();
    int n = drainSince(last_frame_us, batch, DRAIN_N, &anchor);
    Serial.printf("F,%lld,%d,%u\n", (long long)now, n,
                  (unsigned)drops.load(std::memory_order_relaxed));
    for (int i = 0; i < n; i++) {
      Serial.printf("I,%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        (long long)batch[i].t_us,
        batch[i].ax, batch[i].ay, batch[i].az,
        batch[i].gx, batch[i].gy, batch[i].gz);
    }
    last_frame_us = now;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));   // 20 Hz
  }
}

void setup() {
  Serial.begin(921600);                   // 115200 will overflow at 200 Hz
  delay(1000);                            // let USB CDC enumerate
  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("BOOT");
  xTaskCreate(imuTask,   "imu",   4096, nullptr, 5, nullptr);
  xTaskCreate(frameTask, "frame", 8192, nullptr, 4, nullptr);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }