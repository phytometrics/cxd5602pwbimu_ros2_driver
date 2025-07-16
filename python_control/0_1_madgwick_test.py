from ahrs.filters import Madgwick
import numpy as np
from utils.imu import IMUReader

PORT     = "/dev/cu.usbserial-1140" 
BAUDRATE = 115_200
FREQUENCY = 960
GAIN = 0.1  # Madgwick filter gain
imu_reader = IMUReader(port=PORT, baudrate=BAUDRATE)
madgwick = Madgwick(frequency=FREQUENCY, gain=GAIN)




# initial quaternion
q = np.array([1.0, 0.0, 0.0, 0.0])
try:
  with imu_reader as reader:
      for i, (sec, msec, ax, ay, az, gx, gy, gz) in enumerate(reader.stream_data()):
        
        # acc (ax, ay, az)の単位はmeters/second^2
        # gyro (gx, gy, gz)の単位はdegrees/second
        # madgwickが必要とするのは、gyr(rad/s), acc(m/s^2)
        
        # gx gy gzの単位はdegrees/secondのはずなのに、生の値を使うと計算がうまくいく。。。
        gyro_data = np.array([gx, gy, gz]) #* (np.pi / 180.0)  # degrees to radians
        
        acc_data = np.array([ax, ay, az])
        q = madgwick.updateIMU(q, gyr=gyro_data, acc=acc_data)        
        if i % 10 == 0:
          # 小数点3桁まで表示
          print(f"acc: {np.round(acc_data,3)}")
          print(f"gyro: {np.round(gyro_data,3)}")
          print(f"q: {np.round(q, 3)}")
          print("\n")

          # q = process_imu_data(sec, msec, ax, ay, az, gx, gy, gz)
          # roll, pitch, yaw = quaternion_to_euler(q)
          
          # print(f"{sec}.{msec:03d} "
          #       f"acc={ax:+.3f},{ay:+.3f},{az:+.3f} "
          #       f"gyro={gx:+.3f},{gy:+.3f},{gz:+.3f} "
          #       f"q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f}) "
          #       f"rpy=({math.degrees(roll):+6.1f},{math.degrees(pitch):+6.1f},{math.degrees(yaw):+6.1f})")
except KeyboardInterrupt:
    print("\nStopping IMU processing...")
