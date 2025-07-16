/* キャリブレーション今オフにしている
 * Sony Spresense Multi-IMU 改良版ファームウェア
 * - 動作フォーマットは setup() に統一、loop() は空
 * - 可変設定マクロ化
 * - 起動時キャリブレーション
 * - LEDによる状態モニタリング 4灯
 * - シーケンス番号付きフレーム出力
 */

//
// Sony Multi IMUの出力における各軸の方向は：

// X軸 (ax, gx): デバイスの前方方向（フォワード方向）
// Y軸 (ay, gy): デバイスの右側方向（ライト方向）
// Z軸 (az, gz): デバイスの上方向（アップ方向）

// これは右手座標系を採用しており、静止状態では重力により az（Z軸加速度）が約+9.8 m/s² を示すことで上向きであることが確認できます。

#include <Arduino.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include "CRC8.h"

// ── LED 定義 ───────────────────────────
#define LED_STATUS   LED0  // 起動完了インジケータ
#define LED_ACTIVE   LED1  // データ取得中
#define LED_ERROR    LED2  // エラー表示（汎用）
#define LED_FATAL    LED3  // 致命的エラー

// ── 設定マクロ ─────────────────────────
#define SAMPLING_RATE_HZ  960    // 15,30,60,120,240,480,960,1920
#define ACC_RANGE_G       16     // 2,4,8,16
#define GYRO_RANGE_DPS    4000   // 125,250,500,1000,2000,4000
#define FIFO_DEPTH        4      // 1〜4
#define CALIB_SAMPLES     1000   // 起動時バイアス計算サンプル数
#define DROP_INITIAL_MS   50     // 起動時ダミー読み飛ばし(ms)

const char SERIAL_HEADER = 'X';
union float2byte { float f; byte b[4]; };

static cxd5602pwbimu_data_t g_data[FIFO_DEPTH];
static CRC8 crc;
static uint32_t frame_seq = 0;

// バイアス補正値
static float bias_acc[3]  = {0};
static float bias_gyro[3] = {0};

// エラー時に致命的LEDを点滅させる
void fatalError() {
  while (true) {
    digitalWrite(LED_FATAL, HIGH);
    delay(200);
    digitalWrite(LED_FATAL, LOW);
    delay(200);
  }
}

// IMUセンサー初期化
int initIMU() {
  int fd = open("/dev/imu0", O_RDONLY);
  if (fd < 0) return -1;

  if (ioctl(fd, SNIOC_SSAMPRATE, SAMPLING_RATE_HZ) < 0) return -2;
  cxd5602pwbimu_range_t range = { .accel = ACC_RANGE_G, .gyro = GYRO_RANGE_DPS };
  if (ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range) < 0) return -3;
  if (ioctl(fd, SNIOC_SFIFOTHRESH, FIFO_DEPTH) < 0) return -4;
  if (ioctl(fd, SNIOC_ENABLE, 1) < 0) return -5;
  return fd;
}

// 起動時キャリブレーション
void calibrateIMU(int fd) {
  digitalWrite(LED_ACTIVE, HIGH);
  double acc_sum[3]  = {0};
  double gyro_sum[3] = {0};

  uint32_t to_drop = (SAMPLING_RATE_HZ * DROP_INITIAL_MS) / 1000;
  while (to_drop--) read(fd, g_data, sizeof(g_data));

  for (int i = 0; i < CALIB_SAMPLES; ++i) {
    if (read(fd, g_data, sizeof(g_data)) != sizeof(g_data)) continue;
    for (int j = 0; j < FIFO_DEPTH; ++j) {
      acc_sum[0]  += g_data[j].ax;
      acc_sum[1]  += g_data[j].ay;
      acc_sum[2]  += g_data[j].az;
      gyro_sum[0] += g_data[j].gx;
      gyro_sum[1] += g_data[j].gy;
      gyro_sum[2] += g_data[j].gz;
    }
  }
  float factor = 1.0f / (CALIB_SAMPLES * FIFO_DEPTH);
  for (int k = 0; k < 3; ++k) {
    bias_acc[k]  = acc_sum[k]  * factor;
    bias_gyro[k] = gyro_sum[k] * factor;
  }
  digitalWrite(LED_ACTIVE, LOW);
}

// タイムスタンプ取得
void getTimestamp(uint32_t &sec, uint32_t &msec) {
  uint32_t ms = millis();
  sec  = ms / 1000;
  msec = ms % 1000;
}

// メインデータループ
void processLoop(int fd) {
  setvbuf(stdout, NULL, _IONBF, 0);
  digitalWrite(LED_STATUS, HIGH);

  while (true) {
    digitalWrite(LED_ACTIVE, HIGH);
    int ret = read(fd, g_data, sizeof(g_data));
    if (ret != sizeof(g_data)) {
      digitalWrite(LED_ERROR, HIGH);
      continue;
    }

    for (int i = 0; i < FIFO_DEPTH; ++i) {
      float lin_acc[3] = { g_data[i].ax, g_data[i].ay, g_data[i].az };
      float ang_vel[3] = { g_data[i].gx, g_data[i].gy, g_data[i].gz };
    //   float lin_acc[3] = {
    //     g_data[i].ax - bias_acc[0],
    //     g_data[i].ay - bias_acc[1],
    //     g_data[i].az - bias_acc[2]
    //   };
    //   float ang_vel[3] = {
    //     g_data[i].gx - bias_gyro[0],
    //     g_data[i].gy - bias_gyro[1],
    //     g_data[i].gz - bias_gyro[2]
    // };

    uint32_t sec, msec;
    getTimestamp(sec, msec);
    crc.restart();
    crc.add((uint8_t*)&SERIAL_HEADER, 1);
    crc.add((uint8_t*)&sec, 4);
    crc.add((uint8_t*)&msec, 4);
    crc.add((uint8_t*)lin_acc, 12);
    crc.add((uint8_t*)ang_vel, 12);
    uint8_t crc8 = crc.calc();

    printf("%c", SERIAL_HEADER);
    // printf("%c%c%c%c", (char)frame_seq, (char)(frame_seq>>8), (char)(frame_seq>>16), (char)(frame_seq>>24));
    printf("%c%c%c%c", (char)(sec), (char)(sec>>8), (char)(sec>>16), (char)(sec>>24));
    printf("%c%c%c%c", (char)(msec), (char)(msec>>8), (char)(msec>>16), (char)(msec>>24));
      for (int j = 0; j < 3; ++j) {
        float2byte u; u.f = lin_acc[j]; for (int b = 0; b < 4; ++b) printf("%c", u.b[b]);
      }
      for (int j = 0; j < 3; ++j) {
        float2byte u; u.f = ang_vel[j]; for (int b = 0; b < 4; ++b) printf("%c", u.b[b]);
      }
      printf("%c\n", crc8);

      frame_seq++;
    }

    digitalWrite(LED_ACTIVE, LOW);
  }
}

// Arduino エントリポイント
void setup() {
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_ACTIVE, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_FATAL, OUTPUT);

  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_ACTIVE, LOW);
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_FATAL, LOW);

  board_cxd5602pwbimu_initialize(5);
  int fd = initIMU(); if (fd < 0) fatalError();

  calibrateIMU(fd);
  processLoop(fd);
}

void loop() {}
