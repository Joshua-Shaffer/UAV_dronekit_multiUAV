#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
from dronekit import connect, VehicleMode
import time

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--source-num',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
source_id = args.source_num
sitl = None

connection_string = ['/dev/ptyv7', '/dev/ptyv6']
source_id = ['253', '254']

#time.sleep(5)
#Start SITL if no connection string specified
#if not connection_string:
    #import dronekit_sitl
    #sitl = dronekit_sitl.start_default()
#connection_string = 'tcp:127.0.0.1:5770'#'/dev/ttyv9'#'127.0.0.1:14550'
vehicles = list()
for idx, conns in enumerate(connection_string):
    print('Connecting to vehicle on: %s' % connection_string)
    vehicles.append(connect(conns, wait_ready=False, baud=57600 ,source_system=int(source_id[idx])))
    vehicles[idx].wait_ready(True, timeout=90)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    armable_check = False
    while not armable_check:
        print(" Waiting for vehicles to initialise...")
        time.sleep(1)
        armable_check = True
        for quads in vehicles:
            armable_check = armable_check and quads.is_armable


    print("Arming motors")
    # Copter should arm in GUIDED mode
    for quads in vehicles:
        quads.mode = VehicleMode("GUIDED")
        quads.armed = True

    # Confirm vehicle armed before attempting to take off
    armed_check = False
    while not armed_check:
        print(" Waiting for arming...")
        time.sleep(1)
        armed_check = True
        for quads in vehicles:
            armed_check = armed_check and quads.armed

    print("Taking off!")
    for quads in vehicles:
        quads.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        for idx, quads in enumerate(vehicles):
            print(" Altitude of vehicle %d: %d", idx+1, quads.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        altitude_reached_group = True
        for quads in vehicles:
            altitude_reached_group = (altitude_reached_group and
            quads.location.global_relative_frame.alt>=aTargetAltitude*0.95)
        if altitude_reached_group:
            print("Both vehicles reached the target altitude")
            break
        time.sleep(1)

arm_and_takeoff(5)

#print("Set default/target airspeed to 3")
#vehicle.airspeed = 3

#print "Going towards first point for 30 seconds ..."
#point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
#vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(20)

#print "Going towards second point for 30 seconds (groundspeed set to 10 m/s) ..."
#point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
#vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
#time.sleep(30)

print("Returning to Launch")
for quads in vehicles:
    quads.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print("Close vehicle object")
for quads in vehicles:
    quads.close()

# Shut down simulator if it was started.
#if sitl is not None:
#    sitl.stop()
