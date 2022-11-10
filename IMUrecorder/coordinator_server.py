import socket
import struct
import sys
import time

multicast_group = ('224.3.29.71', 10000)

# Create the datagram socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Set a timeout so the socket does not block indefinitely when trying
# to receive data.
# sock.settimeout(0.2)

# Set the time-to-live for messages to 1 so they do not go past the
# local network segment.
ttl = struct.pack('b', 3)
# sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton('10.42.0.1'))


try:
	message = 'start'
	# Send data to the multicast group
	print('sending "%s"' % message)
	sent = sock.sendto(str.encode(message), multicast_group)
	print('sent at time : ',time.time())
	# Look for responses from all recipients
	while True:
		print('waiting to receive')
		try:
			data, server = sock.recvfrom(16)
			print('recieved at time : ',time.time())
		except socket.timeout:
			print('timed out, no more responses')
			break
		else:
			print('received "%s" from %s' % (data, server))

finally:
	print('closing socket')
	sock.close()


