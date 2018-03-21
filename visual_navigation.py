import socket
from struct import *
import atexit

# Close socket
def close_socket(sock):
        sock.close()

# IP address settings (IP address of this machine)
IP_ADDRESS = "192.168.1.4"

# Port settings (if the socket would not bind, pick different port and change it in the vision code as well)
PORT = 5007

# Create socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# command channels
rudder_channel = 1
throttle_channel = 3

# Bind socket to IP address and port
try:
        sock.bind((IP_ADDRESS, PORT))
except socket.error as msg:
        sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((IP_ADDRESS, PORT))

# set socket to nonblocking
sock.settimeout(0.033333)

# Listen indefinitely for the incoming packets (this is blocking)
#atexit.register(close_socket)

# Default throttle nad rudder
throttle = 1500.0
rudder = 1500.0

# Announce that system is ready
print "System is ready!"

while True:
        try:
                # Receive throttle (it is double so receive 8 B)
                package_bin, addr = sock.recvfrom(16)
                
                # Unpack binary data to double
                package = unpack('dd', package_bin)

                # Compute rudder and throttle commands
                throttle = package[0]*400+1500
                rudder = package[1]*400+1500
                
        except socket.timeout:
                #print('Socket timeout')
                pass

        # Send commands to EMILY
        #'''
        Script.SendRC(rudder_channel,rudder,True)
        Script.SendRC(rudder_channel,rudder,False)
        Script.SendRC(throttle_channel,throttle,True)
        Script.SendRC(throttle_channel,throttle,False)
        #'''

        # Print throttle formatted as float
        #print 'Throttle %F' % throttle+'  Rudder %F' % rudder