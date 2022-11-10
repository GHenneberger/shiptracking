import time

import struct

import numpy as np
import signal,os
import errno

import socket
import sys


multicast_group = '224.3.29.71'
server_address = ('', 10000)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(server_address)

group = socket.inet_aton(multicast_group)
mreq = struct.pack('4sL', group, socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

## recieve UDP message
while True:
    print('waiting to receive message')
    data, address = sock.recvfrom(1024)
    
    print('received %s bytes from %s' % (len(data), address))
    print(data.decode())

    print('sending acknowledgement to', address)
    sock.sendto('ack', address)