"""
Spresense CXD5602 Multi-IMU 受信スクリプト  (CRC 検証付き)

フレーム構造 (34 byte 固定)
  0     : 'X'                         ヘッダ
  1-4   : uint32  sec                計測時刻 (秒)
  5-8   : uint32  usec               計測時刻 (マイクロ秒)
  9-20  : float32 ax,ay,az           加速度  [m/s²] (左手座標、ファームで符号反転済)
 21-32  : float32 gx,gy,gz           角速度  [deg/s] (一部符号反転済)
 33     : uint8   CRC8               Dallas/Maxim 方式 (poly 0x31, init 0x00)
 34-35 : '\r''\n'                    フレーム区切り   ←★ここが 2 byte

合計 36 byte が 115200 bps で連続送信される。
"""

import struct
import serial
import crc8
import signal
import sys
import atexit
from loguru import logger

# ========= 環境設定 =========================================
PORT     = "/dev/cu.usbserial-1140"   # ← Spresense のポート名に変更
BAUDRATE = 115_200
TIMEOUT  = 1.0                        # [s] read() タイムアウト
# ===========================================================

# 34 byte ペイロードを unpack するための Struct
STRUCT_PAYLOAD = struct.Struct("<cII6fB")   # 'X' sec usec ax ay az gx gy gz crc
FRAME_SIZE     = 36                         # 34 + CRLF(2)

def crc8_maxim(data: bytes) -> int:
    """Dallas/Maxim CRC-8 (poly=0x31, init=0x00, refIn/Out=True, xorOut=0x00)"""
    h = crc8.crc8()          # pip の crc8 ライブラリ (デフォルトが Dallas/Maxim)
    h.update(data)
    return h.digest()[0]

# Global variable for serial port cleanup
_serial_port = None

def cleanup_serial():
    """Clean up serial port resources"""
    global _serial_port
    if _serial_port and _serial_port.is_open:
        logger.info("Closing serial port...")
        _serial_port.close()
        _serial_port = None

def signal_handler(signum, frame):
    """Handle SIGINT and SIGTERM signals"""
    logger.info(f"Received signal {signum}, shutting down...")
    cleanup_serial()
    sys.exit(0)

class IMUReader:
    def __init__(self, port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self._setup_signal_handlers()
        
    def _setup_signal_handlers(self):
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        atexit.register(self.close)
    
    def _signal_handler(self, signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        self.close()
        sys.exit(0)
        
    def open(self):
        if self.serial_port is None or not self.serial_port.is_open:
            logger.info(f"Open {self.port} @ {self.baudrate} bps")
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.serial_port.reset_input_buffer()
            
    def close(self):
        if self.serial_port and self.serial_port.is_open:
            logger.info("Closing serial port...")
            self.serial_port.close()
            self.serial_port = None
            
    def read_frame(self):
        if not self.serial_port or not self.serial_port.is_open:
            raise RuntimeError("Serial port not open. Call open() first.")
            
        buf = self.serial_port.read(FRAME_SIZE)
        if len(buf) != FRAME_SIZE:
            return None
            
        payload, crlf = buf[:-2], buf[-2:]
        if crlf != b"\r\n":
            self.serial_port.read(1)
            return None
            
        try:
            header, sec, usec, *vals, crc_recv = STRUCT_PAYLOAD.unpack(payload)
        except struct.error:
            return None
            
        if header != b"X":
            return None
            
        crc_calc = crc8_maxim(payload[:-1])
        if crc_calc != crc_recv:
            logger.warning("CRC error")
            return None
            
        ax, ay, az, gx, gy, gz = vals
        return sec, usec, ax, ay, az, gx, gy, gz
        
    def stream_data(self):
        self.open()
        try:
            while True:
                data = self.read_frame()
                if data is not None:
                    yield data
        except serial.SerialException as e:
            logger.error(f"Serial error: {e}")
            self.close()
            raise
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            self.close()
            raise
            
    def __enter__(self):
        self.open()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

def get_imu_raw_output():
    global _serial_port    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Register cleanup function for normal exit
    atexit.register(cleanup_serial)
    
    logger.info(f"Open {PORT} @ {BAUDRATE} bps")
    
    try:
        _serial_port = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        _serial_port.reset_input_buffer()          # 古いゴミを捨てる
        
        while True:
            buf = _serial_port.read(FRAME_SIZE)
            # print(buf)
            if len(buf) != FRAME_SIZE:
                continue                  # タイムアウト → 次ループ

            payload, crlf = buf[:-2], buf[-2:]      # 34 byte + CRLF
            if crlf != b"\r\n":                     # 区切りが違えば同期ずれ
                _serial_port.read(1)                         # 1 byte 捨てて復旧試行
                continue

            try:
                header, sec, usec, *vals, crc_recv = STRUCT_PAYLOAD.unpack(payload)
            except struct.error:
                continue                            # サイズ不整合 → 次

            if header != b"X":
                continue                            # ヘッダずれ

            crc_calc = crc8_maxim(payload[:-1])     # CRC 計算 (CRC バイト除く)
            if crc_calc != crc_recv:
                logger.warning("CRC error, skiping frame")
                continue

            ax, ay, az, gx, gy, gz = vals
            # print(f"{sec}.{usec:03d}  "
            #       f"acc=({ax:+.3f},{ay:+.3f},{az:+.3f})  "
            #       f"gyro=({gx:+.3f},{gy:+.3f},{gz:+.3f})")
            return sec, usec, ax, ay, az, gx, gy, gz
                  
    except serial.SerialException as e:
        logger.error(f"Serial error: {e}")
        cleanup_serial()
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        cleanup_serial()
        sys.exit(1)
