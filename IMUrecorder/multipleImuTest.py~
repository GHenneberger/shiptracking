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

from paramiko import SSHClient
from scp import SCPClient



multicast_group = '224.3.29.71'
server_address = ('', 10000)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(server_address)

group = socket.inet_aton(multicast_group)
mreq = struct.pack('4sL', group, socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)



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


# signal.signal(signal.SIGALRM, _handle_timeout)


## recieve UDP message
while True:
    print('waiting to receive message')
    data, address = sock.recvfrom(1024)
    
    print('received %s bytes from %s' % (len(data), address))
    print(data.decode(), " at time: ",time.time())

    print('sending acknowledgement to', address)
    sock.sendto(str.encode('IMUs started'), address)

    print('*',data.decode(),'*')
    print('start' in data.decode())
    if 'start' in data.decode():
        break

sock.setblocking(0)


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

    if True: #If true, will display the current imu data
        t1 = time.monotonic()
        # signal.setitimer(signal.ITIMER_REAL,0.01)
        try:
            
            # aa1 = (0,0,0,0,0,0)
            a1 = icm1.acceleration
            g1 = icm1.gyro

            X1[cnt1,0]=time.time()
            X1[cnt1,1:4]=a1
            X1[cnt1,4:]=g1
            cnt1+=1

        except:
            print("aa1 error with = ",time.monotonic()-t1)
            # aa1=(0,0,0,0,0,0)
        # finally:
        # signal.setitimer(signal.ITIMER_REAL,0)

        # time.sleep(0.005)
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

        # print("Board Acceleration 1: X:%.4f, Y: %.4f, Z: %.4f m/s^2" % (a1))
        #print("Board Acceleration 2: X:%.4f, Y: %.4f, Z: %.4f m/s^2" % (aa2[0:3]))

        # print("Board Gyro 1 X:%.3f, Y: %.3f, Z: %.3f rads/s" % (g1))
        #print("Board Gyro 2 X:%.3f, Y: %.3f, Z: %.3f rads/s" % (aa2[3:]))

        #print("")

        #print("External Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (icm2.acceleration))
        #print("External Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (icm2.gyro))
        #print("")
        #time.sleep(0.5)
        
    counter = counter + 1
    if time.time() - t > 10: #Update the sample frequency every 5 seconds
        hz.append( counter/(time.time()-t)) #Calculate sample rate in Hz
        print("Rate: %s hz" % np.mean(hz))
        # print("")
        counter = 0
        t = time.time()
    
    if cnt1==N or cnt2==N:
        break

    time.sleep(0.005)

    try:
        data, address = sock.recvfrom(1024)
        if 'stop' in data.decode():
            break
    except:
        pass

        

print("mean freq = ",np.mean(hz))
np.savez("IMUdata.npz",X1=X1,X2=X2)

d= np.diff(X1[:,0])
print("X1 : ", min(d),max(d),np.mean(d),sum(d>1.5*np.mean(d)))

d= np.diff(X2[:,0])
print("X2 : ", min(d),max(d),np.mean(d),sum(d>1.5*np.mean(d)))


ssh = SSHClient()
ssh.load_system_host_keys()
ssh.connect(address[0],username='nagnanmus')

with SCPClient(ssh.get_transport()) as scp:
    scp.put("IMUdata.npz")
