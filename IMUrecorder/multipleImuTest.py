# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
 
import time
import board
import busio
import struct
import adafruit_icm20x
import adafruit_bus_device.i2c_device as i2c_device
import numpy as np
import signal,os
import errno

import socket
import sys








class TimeoutError(Exception):
    pass

def _handle_timeout(signum, frame):
    raise TimeoutError(os.strerror(errno.ETIME))

i2c = busio.I2C(board.SCL, board.SDA,frequency=400000)


icm1 = adafruit_icm20x.ICM20649(i2c,address=0x68) #The imu mounted to the rpi is at the default address of 0x68
icm2 = adafruit_icm20x.ICM20649(i2c,address=0x69) #The imu attached to the extention wire has been shiffted to 0x69 via the jumper



#icm1._low_power = False
#icm1._i2c_master_cycle_en = False
#icm1._gyro_cycle_en = False


icm1.accel_dlpf_cutoff = 7 
icm1.gyro_dlpf_cutoff = 7 
icm2.accel_dlpf_cutoff = 7 
icm2.gyro_dlpf_cutoff = 7 


icm1._accel_dlpf_enable  = False
icm1._gyro_dlpf_enable = False
icm2._accel_dlpf_enable  = False
icm2._gyro_dlpf_enable = False

#icm1.accelerometer_data_rate_divisor = 10
#icm1.gyro_data_rate_divisor = 10

icm1.accelerometer_data_rate=150
icm1.gyro_data_rate=150
icm2.accelerometer_data_rate=150
icm2.gyro_data_rate=150

#icm1.reset()
#icm1.i2c_device = i2c_device.I2CDevice(i2c, 0x69)
#icm1.initialize()

print("Data Rate:", icm1.accelerometer_data_rate_divisor)

time.sleep(2)

N=10000
X1 = np.zeros((N,7),dtype=np.float64)
X2 = np.zeros((N,7),dtype=np.float64)





t = time.time()
counter = 0
cnt1=0
cnt2=0
hz=[]

while True:
    #Record the data in variables for smaple rate testing
    #accel1 = icm1.acceleration
    #gyro1 = icm1.gyro
    #accel2 = icm2.acceleration
    #gyro2 = icm2.gyro


    t1 = time.monotonic()

    try:

        a1 = icm1.acceleration
        g1 = icm1.gyro

        X1[cnt1,0]=time.time()
        X1[cnt1,1:4]=a1
        X1[cnt1,4:]=g1
        cnt1+=1
        print(a1,g1)
    except:
        print("aa1 error with = ",time.monotonic()-t1)


    t2 = time.monotonic()
    try:
        a2 = icm2.acceleration
        g2 = icm2.gyro

        X2[cnt2,0]=time.time()
        X2[cnt2,1:4]=a2
        X2[cnt2,4:]=g2
        cnt2+=1
    except:
        print("aa2 error with = ",time.monotonic()-t2)
        # aa2=(0,0,0,0,0,0)


     
    
    
    if cnt1==N or cnt2==N:
        break

    time.sleep(0.005)

