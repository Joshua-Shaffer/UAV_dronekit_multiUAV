import subprocess
import serial
import threading
from pymavlink import mavutil
from mavlink_hub import MAVLinkHub

MAX_READ_SIZE = 256
'''def forward_to_serial():
    while True:
        data = 'hello'#proc.stdout.read1(MAX_READ_SIZE)
        if not data:
            break  # EOF
        ser.write(data)
        input('wait...')'''

'''def forward_to_drone():
    while True:
        msg = ser2.read()
        #in_bin = ser.read()
        in_hex = hex(int(msg.encode('hex'), 16))
        if in_hex is not None:
            #data_drone.write(str(msg))
            #print('From kit to drone')
            print(msg)
        #proc.stdin.write(ser.read())
        #proc.stdin.flush()

def forward_to_kit():
    while True:
        msg = data_drone.recv_match()
        if msg is not None:
            #data_kit.write(str(msg))
            print('From drone to kit')
            print(msg)
        #proc.stdin.write(ser.read())
        #proc.stdin.flush()'''

def valid_route_hub2(srcSystem, j):
    #print(srcSystem)
    #print('routing function')
    if srcSystem == 1:
        if j.addr == '/dev/ttyv6' or j.addr == '/dev/ttyv8':
            if j.addr == '/dev/ttyv8':
                print('Go to GCS')
            return True#True
    if srcSystem == 2:
        if j.addr == '/dev/ttyv7' or j.addr == '/dev/ttyv8':
            if j.addr == '/dev/ttyv8':
                print('Go to GCS')
            return False#True
    if srcSystem == 255 or srcSystem == 254 or srcSystem == 253:
        if j.addr == '/dev/cu.usbserial-AL02K12Z':
            return True
    return False

def startMAV_hub(conn, valid, conn_baud):
    mav_hub = MAVLinkHub(conn, valid, conn_baud)
    mav_hub.run()
#ser = serial.Serial('/dev/ttyv9', timeout = 100)
#input('wait...')
#ser2 = serial.Serial('/dev/ptyv9', timeout = 100)
#ser2.baudrate = 115200
#data_kit = mavutil.mavlink_connection('/dev/ptyv9', write=True)
#data_drone = mavutil.mavlink_connection('tcp:127.0.0.1:5770', write=True)
connections1 = ['/dev/ttyv6', '/dev/ttyv7', '/dev/cu.usbserial-AL02K12Z']
connections1_baud = [57600, 57600, 57600]
hub1 = threading.Thread(target=startMAV_hub(connections1, valid_route_hub2, connections1_baud))
hub1.start()


#proc = subprocess.Popen("cat", stdout=subprocess.PIPE, stdin=subprocess.PIPE)

#fwc = threading.Thread(target = forward_to_kit)
#fwc.start()

#input('wait...')
#forward_to_drone()
