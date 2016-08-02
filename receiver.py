import socket
from struct import *

# IP address settings
IP_ADDRESS = "192.168.1.4"

# Port settings
PORT = 5005

# Create socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind socket to IP address and port
sock.bind((IP_ADDRESS, PORT))

# Listen indefinitely for the incoming packets
while True:

	# Receive throttle (it is double so receive 8 B)
	package_bin, addr = sock.recvfrom(16)
	
	# Unpack binary data to double
	package = unpack('dd', package_bin)
        
    # Print throttle formatted as float
	print 'Throttle %F' % package[0]
	print 'Rudder %F' % package[1]