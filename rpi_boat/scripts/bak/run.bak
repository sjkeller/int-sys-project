#!/bin/sh

import numpy as np
from dronekit import connect

cfg = np.loadtxt('boat.cfg')

vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200, use_native=False)

vehicle.parameters['RUDDER_MID'] = cfg[0]
print("RUDDER_MID",vehicle.parameters['RUDDER_MID'])

vehicle.parameters['SAIL_MID'] = cfg[1]
print("SAIL_MID",vehicle.parameters['SAIL_MID'])

vehicle.mode = 'MANUAL'
print(vehicle.mode)
