import socket
from struct import *

# IP address settings
IP_ADDRESS = "127.0.0.1"

# Port settings
PORT = 5005

# Create socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind socket to IP address and port
sock.bind((IP_ADDRESS, PORT))

# Listen indefinitely for the incoming packets
while True:

	# Receive throttle (it is double so receive 8 B)
	throttle_bin, addr = sock.recvfrom(8)
	
	# Unpack binary data to double
	throttle = unpack('d', throttle_bin)

	# Print throttle formatted as float
	print 'Throttle %F' % throttle

	# Receive rudder
	rudder_bin, addr = sock.recvfrom(8)

	# Unpack binary data to double
	rudder = unpack('d', rudder_bin)

	# Print rudder formatted as float
	print 'Rudder %F' % rudder