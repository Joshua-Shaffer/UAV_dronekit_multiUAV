#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
vehicle_state.py:

Demonstrates how to get and set vehicle state and parameter information,
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""
from __future__ import print_function
from dronekit import connect, VehicleMode
import time

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    #import dronekit_sitl
    #sitl = dronekit_sitl.start_default()
    connection_string = 'tcp:127.0.0.1:5760'#'/dev/ttyv9'#'127.0.0.1:14550'
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=115200)


# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
#print("\nConnecting to vehicle on: %s" % connection_string)
#vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')
print(vehicle.parameters['SYSID_THISMAV'])
vehicle.parameters['SYSID_THISMAV'] = 2
print(vehicle.parameters['SYSID_THISMAV'])
vehicle.close()
