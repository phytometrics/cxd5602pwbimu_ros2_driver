/* Sony Spresense Multi-IMU ノンキャリブレーション版
 * - キャリブレーション処理を完全削除
 * - シンプルなLED状態表示
 * - 生データをそのまま出力
 * - エラーハンドリング簡素化
 */

//
// Sony Multi IMUの出力における各軸の方向は：
// X軸 (ax, gx): デバイスの前方方向（フォワード方向）
// Y軸 (ay, gy): デバイスの右側方向（ライト方向）
// Z軸 (az, gz): デバイスの上方向（アップ方向）
// 右手座標系、静止状態では az が約+9.8 m/s²

#include <Arduino.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include "CRC8.h"

// ── LED 定義（シンプル版）──────────────────
#define LED_POWER    LED0  // 電源ON/システム起動完了
#define LED_RUNNING  LED1  // データ取得中（点滅）
#define LED_ERROR    LED2  // エラー発生時
#define LED_DATA     LED3  // データ送信中（高速点滅）

// ── 設定マクロ ─────────────────────────
#define SAMPLING_RATE_HZ  60    // 15,30,60,120,240,480,960,1920
#define ACC_RANGE_G       16     // 2,4,8,16
#define GYRO_RANGE_DPS    4000   // 125,250,500,1000,2000,4000
#define FIFO_DEPTH        1      // 1〜4
#define STABILIZATION_MS  500    // センサー安定化待ち時間（ミリ秒）

const char SERIAL_HEADER = 'X';
union float2byte { float f; byte b[4]; };

static cxd5602pwbimu_data_t g_data[FIFO_DEPTH];
static CRC8 crc;
static uint32_t frame_seq = 0;

// エラー時の処理
void handleError() {
  digitalWrite(LED_ERROR, HIGH);
  delay(100);
  digitalWrite(LED_ERROR, LOW);
  delay(100);
}

// 致命的エラー時の処理
void fatalError() {
  while (true) {
    digitalWrite(LED_ERROR, HIGH);
    digitalWrite(LED_POWER, HIGH);
    delay(500);
    digitalWrite(LED_ERROR, LOW);
    digitalWrite(LED_POWER, LOW);
    delay(500);
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

// センサー安定化処理
void stabilizeIMU(int fd) {
  digitalWrite(LED_RUNNING, HIGH);
  digitalWrite(LED_DATA, HIGH);
  
  uint32_t start_time = millis();
  uint32_t current_time = start_time;
  
  // 安定化期間中はデータを読み捨てる
  while ((current_time - start_time) < STABILIZATION_MS) {
    // データを読み捨て
    read(fd, g_data, sizeof(g_data));
    
    // LED点滅で安定化中であることを示す
    if ((current_time - start_time) % 100 < 50) {
      digitalWrite(LED_DATA, HIGH);
    } else {
      digitalWrite(LED_DATA, LOW);
    }
    
    delay(10); // 短い待機
    current_time = millis();
  }
  
  digitalWrite(LED_RUNNING, LOW);
  digitalWrite(LED_DATA, LOW);
}
// タイムスタンプ取得
void getTimestamp(uint32_t &sec, uint32_t &usec) {
  uint32_t us = micros();
  sec  = us / 1000000;
  usec = us % 1000000;
}

// メインデータループ
void processLoop(int fd) {
  setvbuf(stdout, NULL, _IONBF, 0);
  
  // システム起動完了
  digitalWrite(LED_POWER, HIGH);
  
  uint32_t led_toggle_counter = 0;
  const uint32_t LED_TOGGLE_INTERVAL = 50; // データLED点滅間隔

  while (true) {
    // データ取得中表示
    digitalWrite(LED_RUNNING, HIGH);
    
    int ret = read(fd, g_data, sizeof(g_data));
    if (ret != sizeof(g_data)) {
      handleError();
      continue;
    }

    // データ送信LED制御
    led_toggle_counter++;
    if (led_toggle_counter >= LED_TOGGLE_INTERVAL) {
      digitalWrite(LED_DATA, !digitalRead(LED_DATA));
      led_toggle_counter = 0;
    }

    for (int i = 0; i < FIFO_DEPTH; ++i) {
      // 生データをそのまま使用（キャリブレーションなし）
      float lin_acc[3] = { g_data[i].ax, g_data[i].ay, g_data[i].az };
      float ang_vel[3] = { g_data[i].gx, g_data[i].gy, g_data[i].gz };

      uint32_t sec, usec;
      getTimestamp(sec, usec);
      
      // CRC計算
      crc.restart();
      crc.add((uint8_t*)&SERIAL_HEADER, 1);
      crc.add((uint8_t*)&sec, 4);
      crc.add((uint8_t*)&usec, 4);
      crc.add((uint8_t*)lin_acc, 12);
      crc.add((uint8_t*)ang_vel, 12);
      uint8_t crc8 = crc.calc();

      // データ出力
      printf("%c", SERIAL_HEADER);
      printf("%c%c%c%c", (char)(sec), (char)(sec>>8), (char)(sec>>16), (char)(sec>>24));
      printf("%c%c%c%c", (char)(usec), (char)(usec>>8), (char)(usec>>16), (char)(usec>>24));
      
      // 加速度データ
      for (int j = 0; j < 3; ++j) {
        float2byte u; 
        u.f = lin_acc[j]; 
        for (int b = 0; b < 4; ++b) printf("%c", u.b[b]);
      }
      
      // 角速度データ
      for (int j = 0; j < 3; ++j) {
        float2byte u; 
        u.f = ang_vel[j]; 
        for (int b = 0; b < 4; ++b) printf("%c", u.b[b]);
      }
      
      printf("%c\n", crc8);
      frame_seq++;
    }

    digitalWrite(LED_RUNNING, LOW);
  }
}

// Arduino エントリポイント
void setup() {
  // LED初期化
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_RUNNING, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_DATA, OUTPUT);

  // 全LED消灯
  digitalWrite(LED_POWER, LOW);
  digitalWrite(LED_RUNNING, LOW);
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_DATA, LOW);

  // Multi-IMUボード初期化
  board_cxd5602pwbimu_initialize(5);
  
  // IMU初期化
  int fd = initIMU(); 
  if (fd < 0) {
    fatalError(); // 初期化失敗時は致命的エラー
  }

  // センサー安定化待ち
  stabilizeIMU(fd);

  // メインループ開始
  processLoop(fd);
}

void loop() {
  // 全処理をsetup()で実行
}