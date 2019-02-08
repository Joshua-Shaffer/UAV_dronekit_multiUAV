import subprocess
import serial
import threading
from pymavlink import mavutil
from mavlink_hub import MAVLinkHub

MAX_READ_SIZE = 256

def valid_route_hub(srcSystem, j):
    #print(srcSystem)
    #print('routing function')
    if srcSystem == 254 or srcSystem == 255:
        return True
    if srcSystem == 1:
        return True
    return False

def startMAV_hub(conn, valid):
    mav_hub = MAVLinkHub(conn, valid)
    mav_hub.run()
#ser = serial.Serial('/dev/ttyv9', timeout = 100)
#input('wait...')
#ser2 = serial.Serial('/dev/ptyv9', timeout = 100)
#ser2.baudrate = 115200
#data_kit = mavutil.mavlink_connection('/dev/ptyv9', write=True)
#data_drone = mavutil.mavlink_connection('tcp:127.0.0.1:5770', write=True)
connections = ['/dev/ttyAMA0', 'tcp:127.0.0.1:5760']
hub = threading.Thread(target=startMAV_hub(connections, valid_route_hub))
hub.start()


#proc = subprocess.Popen("cat", stdout=subprocess.PIPE, stdin=subprocess.PIPE)

#fwc = threading.Thread(target = forward_to_kit)
#fwc.start()

#input('wait...')
#forward_to_drone()
