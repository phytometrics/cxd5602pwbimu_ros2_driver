#!/usr/bin/env python3
"""
Spresense CXD5602 Multi-IMU 受信スクリプト  (CRC 検証付き)

フレーム構造 (34 byte 固定)
  0     : 'X'                         ヘッダ
  1-4   : uint32  sec                計測時刻 (秒)
  5-8   : uint32  msec               計測時刻 (ミリ秒)
  9-20  : float32 ax,ay,az           加速度  [m/s²] (左手座標、ファームで符号反転済)
 21-32  : float32 gx,gy,gz           角速度  [deg/s] (一部符号反転済)
 33     : uint8   CRC8               Dallas/Maxim 方式 (poly 0x31, init 0x00)
 34-35 : '\r''\n'                    フレーム区切り   ←★ここが 2 byte

合計 36 byte が 115200 bps で連続送信される。
"""

import struct
import serial
import crc8

# ========= 環境設定 =========================================
PORT     = "/dev/cu.usbserial-1130"   # ← Spresense のポート名に変更
BAUDRATE = 115_200
TIMEOUT  = 1.0                        # [s] read() タイムアウト
# ===========================================================

# 34 byte ペイロードを unpack するための Struct
STRUCT_PAYLOAD = struct.Struct("<cII6fB")   # 'X' sec msec ax ay az gx gy gz crc
FRAME_SIZE     = 36                         # 34 + CRLF(2)

def crc8_maxim(data: bytes) -> int:
    """Dallas/Maxim CRC-8 (poly=0x31, init=0x00, refIn/Out=True, xorOut=0x00)"""
    h = crc8.crc8()          # pip の crc8 ライブラリ (デフォルトが Dallas/Maxim)
    h.update(data)
    return h.digest()[0]

def main() -> None:
    print(f"Open {PORT} @ {BAUDRATE} bps")
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    ser.reset_input_buffer()          # 古いゴミを捨てる

    while True:
        buf = ser.read(FRAME_SIZE)
        if len(buf) != FRAME_SIZE:
            continue                  # タイムアウト → 次ループ

        payload, crlf = buf[:-2], buf[-2:]      # 34 byte + CRLF
        if crlf != b"\r\n":                     # 区切りが違えば同期ずれ
            ser.read(1)                         # 1 byte 捨てて復旧試行
            continue

        try:
            header, sec, msec, *vals, crc_recv = STRUCT_PAYLOAD.unpack(payload)
        except struct.error:
            continue                            # サイズ不整合 → 次

        if header != b"X":
            continue                            # ヘッダずれ

        crc_calc = crc8_maxim(payload[:-1])     # CRC 計算 (CRC バイト除く)
        if crc_calc != crc_recv:
            print("CRC error")
            continue

        ax, ay, az, gx, gy, gz = vals
        print(f"{sec}.{msec:03d}  "
              f"acc=({ax:+.3f},{ay:+.3f},{az:+.3f})  "
              f"gyro=({gx:+.3f},{gy:+.3f},{gz:+.3f})")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
