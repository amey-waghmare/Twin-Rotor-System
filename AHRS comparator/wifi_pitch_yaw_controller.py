

import numpy as np
import matplotlib.pyplot as plt
import math

dt_angles = 0.01

import sys

import time
import board
import serial
import adafruit_bno055
import busio



from quat2euler import quat2euler_angle

from ahrs.filters import Mahony

## Mahony
#from mahony_ahrs import Mahony    ## Updated library with corrected Integrator Implementation
from ahrs import Quaternion
## Without disturbance
#orientation = Mahony(frequency = 100.0,k_P = 45, k_I = 30)
orientation = Mahony(frequency = 100.0,k_P = 7, k_I = 5) ### Working great for steady state


## BNO055
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)



epochs = 15000
Q = np.tile([1.0, 0.0,0.0,0.0], (epochs,1))
eul = []

err = []

angles = []
t = 0

while True:
    
    yaw_bno, roll_bno, pitch_bno = sensor.euler
    a = sensor.acceleration
    g = sensor.gyro
    m = sensor.magnetic
    if yaw_bno != None or roll_bno != None or pitch_bno != None or a[0] != None or a[1] != None or a[2] != None or g[0] != None or g[1] != None or g[2] != None or m[0] != None or m[1] != None or m[2] != None:
        #print (a, g, m, yaw_bno, roll_bno, pitch_bno)
        
        
        angles.append([pitch_bno, roll_bno, yaw_bno])
        #Q[t] = orientation.updateMARG(Q[t-1], gyr = g, acc = a, mag = m)    
        Q[t] = orientation.updateIMU(Q[t-1], gyr = g, acc = a)        
        meas_pitch, meas_roll, meas_yaw = quat2euler_angle(Q[t,0], Q[t,1], Q[t,2], Q[t,3])
        
        print("Pitch_BNO: {:.2f}\t Manhony: {:.2f}\tRoll_BNO: {:.2f}\t Manhony: {:.2f}\tYaw_BNO: {:.2f}\t Manhony: {:.2f}\t".format(pitch_bno, meas_pitch, roll_bno, meas_roll, yaw_bno, meas_yaw))
    t = t + 1
    if t == epochs:
        break

angles = np.array(angles)
np.savetxt("comparision.csv", angles, delimiter = ",")

fig, axs = plt.subplots(3)
fig.suptitle("Euler angles")
axs[0].plot(angles[1:,0])
axs[0].grid()
axs[0].legend("Pitch")
axs[0].set_ylabel("Pitch (degree)")
axs[1].plot(angles[1:,1])
axs[1].legend("Roll")
axs[1].set_ylabel("Roll (degree)")
axs[1].grid()
axs[2].plot(angles[1:,2])
axs[2].legend("Yaw")
axs[2].set_ylabel("Yaw (degree)")

plt.grid()
plt.legend()
plt.savefig("latest_results.png")
plt.show()
