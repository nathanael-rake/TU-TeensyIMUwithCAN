# -*- coding: utf-8 -*-
"""
Created on Wed Aug 31 21:05:47 2016

@author: home
"""

import serial
import matplotlib.pyplot as plt

plotUpdate = 20*4 #number of timesteps between plot updates
port = 'COM4'
baud = 115200

#open serial port
ser = serial.Serial(port, baud)

#initialize arrays
vals = [None]*6 #samples from a single timestep
xAccel = [] #all x acceleration values
yAccel = [] #all y acceleration values
zAccel = [] #all z acceleration values
rollRate = [] #all roll rates
pitchRate = [] #all pitch rates
yawRate = [] #all yaw Rates


#begin loop
n = 0 #itteration counter
try:
    while True:
        rawLine = ser.readline();
        stringLine = rawLine.decode('utf-8')
        stringLine = stringLine.strip("\r")
        stringLine = stringLine.strip("\n")
        stringArray = stringLine.split()
        for i in range(len(stringArray)):
            vals[i] = float(stringArray[i])
        xAccel.append(vals[0])
        yAccel.append(vals[1])
        zAccel.append(vals[2])
        rollRate.append(vals[3])
        pitchRate.append(vals[4])
        yawRate.append(vals[5])
        
        #update plot if necessary
        if n%plotUpdate == 0:
            plt.plot(xAccel)
            plt.plot(yAccel)
            plt.plot(zAccel)
            plt.plot(rollRate)
            plt.plot(pitchRate)
            plt.plot(yawRate)
            plt.show()
       
except KeyboardInterrupt:
    ser.close()