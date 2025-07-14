/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>

#include "CRC8.h"
#include "CRC.h"

#define CXD5602PWBIMU_DRIVER_DEVPATH      "/dev/imu0"
#define MAX_NFIFO 4
const char SERIAL_HEADER = 'X';

union float2byte {
  float f;
  byte b[4];
};


static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
CRC8 crc;

static void get_time(int32_t &sec, int32_t &msec)
{
  unsigned long ms = millis();
  sec = ms / 1000;
  msec = ms % 1000;
}

static void interpolate_timestamp(int32_t base_sec, int32_t base_msec, int sample_idx, int32_t &out_sec, int32_t &out_msec)
{
  // 960Hz = 1.041667ms per sample
  int32_t sample_offset_us = (sample_idx * 1041667) / 1000; // microseconds
  int32_t total_us = base_msec * 1000 + sample_offset_us;
  
  out_sec = base_sec + (total_us / 1000000);
  out_msec = (total_us % 1000000) / 1000;
}

static int start_sensing(int fd, int rate, int adrange, int gdrange, int nfifos)
{
  cxd5602pwbimu_range_t range;
  int ret;

  ret = ioctl(fd, SNIOC_SSAMPRATE, rate);
  if (ret != 0) {
    printf("ERROR: Set sampling rate failed: %d\n", ret);
    return -1;
  }

  range.accel = adrange;
  range.gyro = gdrange;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret != 0) {
    printf("ERROR: Set dynamic range failed: %d\n", ret);
    return -1;
  }

  ret = ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  if (ret != 0) {
    printf("ERROR: Set FIFO threshold failed: %d\n", ret);
    return -1;
  }

  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret != 0) {
    printf("ERROR: Enable sensor failed: %d\n", ret);
    return -1;
  }

  return 0;
}

static int drop_50msdata(int fd, int samprate)
{
  int cnt = samprate / 20; /* 50ms skip */

  cnt = ((cnt + MAX_NFIFO - 1) / MAX_NFIFO) * MAX_NFIFO;
  if (cnt == 0) cnt = MAX_NFIFO;

  while (cnt)
    {
      read(fd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
      cnt -= MAX_NFIFO;
    }

  return 0;
}

void setup() {
  int devfd;
  int ret;
  
  ret = board_cxd5602pwbimu_initialize(5);
  if (ret != 0) {
    printf("ERROR: Board initialization failed: %d\n", ret);
    return;
  }

  devfd = open(CXD5602PWBIMU_DRIVER_DEVPATH, O_RDONLY);
  if (devfd < 0) {
    printf("ERROR: Could not open device: %d\n", devfd);
    return;
  }

  ret = start_sensing(devfd, 960, 16, 4000, MAX_NFIFO);
  if (ret < 0) {
    printf("ERROR: Sensor initialization failed\n");
    close(devfd);
    return;
  }

  drop_50msdata(devfd, 960);
  delay(2000);

  printf("IMU sensor initialized successfully\n");

  int32_t sec = 0;
  int32_t msec = 0;

  float linear_acceleration[3] = {0};
  float angular_velocity[3] = {0};
  while(1) {
    int ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    if(ret == sizeof(g_data[0]) * MAX_NFIFO) {
      // Get base timestamp for this batch
      int32_t base_sec, base_msec;
      get_time(base_sec, base_msec);
      
      for(int i=0; i<MAX_NFIFO; i++) {
        // Use hardware timestamp if available, otherwise interpolate
        if (g_data[i].timestamp > 0) {
          sec = g_data[i].timestamp / 1000;
          msec = g_data[i].timestamp % 1000;
        } else {
          interpolate_timestamp(base_sec, base_msec, i, sec, msec);
        }
        
        linear_acceleration[0] = -g_data[i].ax;
        linear_acceleration[1] = -g_data[i].ay;
        linear_acceleration[2] = -g_data[i].az;
        angular_velocity[0] = -g_data[i].gx; 
        angular_velocity[1] = -g_data[i].gy; 
        angular_velocity[2] = g_data[i].gz;

        float2byte f2b_linear_acceleration[3];
        float2byte f2b_angular_velocity[3];
        for(int j=0; j<3; j++) {
          f2b_linear_acceleration[j].f = linear_acceleration[j];
          f2b_angular_velocity[j].f = angular_velocity[j];
        }

        crc.restart();
        crc.add((uint8_t*)&SERIAL_HEADER, 1);
        crc.add((uint8_t*)&sec, 4);
        crc.add((uint8_t*)&msec, 4);
        crc.add((uint8_t*)&linear_acceleration, 12);
        crc.add((uint8_t*)&angular_velocity, 12);
        uint8_t crc8 = crc.calc();

        printf("%c", SERIAL_HEADER);
        printf("%c%c%c%c", (char)(sec), (char)(sec >> 8), (char)(sec >> 16), (char)(sec >> 24));
        printf("%c%c%c%c", (char)(msec), (char)(msec >> 8), (char)(msec >> 16), (char)(msec >> 24));
        for(int k=0; k<3; k++) {
          float2byte f2b;
          f2b.f = linear_acceleration[k];
          for(int j=0; j<4; j++) {
            printf("%c", f2b.b[j]);
          }
        }
        for(int k=0; k<3; k++) {
          float2byte f2b;
          f2b.f = angular_velocity[k];
          for(int j=0; j<4; j++) {
            printf("%c", f2b.b[j]);
          }
        }
        printf("%c\n", crc8);
      }
    } else {
      printf("ERROR: Read failed, expected %d bytes, got %d\n", (int)(sizeof(g_data[0]) * MAX_NFIFO), ret);
      delay(10);
    }
  }
}

void loop() {}
