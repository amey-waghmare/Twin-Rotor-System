
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
orientation = Mahony(frequency = 100.0,k_P = 10, k_I = 8) ### Working great for steady state


## BNO055
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)



epochs = 2000
Q = np.tile([1.0, 0.0,0.0,0.0], (epochs,1))
eul = []

err = []

angles_bno = []
angles_man = []
t = 0

prev_pitch = 0
prev_roll = 0
prev_yaw = 0

while True:
    
    yaw_bno, roll_bno, pitch_bno = sensor.euler
    a = sensor.acceleration
    g = sensor.gyro
    m = sensor.magnetic
    if yaw_bno != None or roll_bno != None or pitch_bno != None:
        if pitch_bno >=360 or pitch_bno <= -360:
            pitch_bno = prev_pitch
        if roll_bno >=360 or roll_bno <= -360:
            roll_bno = prev_pitch
        if yaw_bno > 365 or yaw_bno < -2:
            yaw_bno = prev_yaw
        if a[0] != None or a[1] != None or a[2] != None:
            
            #print (a, g, m, yaw_bno, roll_bno, pitch_bno)
        
        
            angles_bno.append([pitch_bno, roll_bno, yaw_bno])
            #Q[t] = orientation.updateMARG(Q[t-1], gyr = g, acc = a, mag = m)    
            Q[t] = orientation.updateIMU(Q[t-1], gyr = g, acc = a)        
            meas_pitch, meas_roll, meas_yaw = quat2euler_angle(Q[t,0], Q[t,1], Q[t,2], Q[t,3])
            meas_pitch = -1*meas_pitch
            meas_roll = -1*meas_roll
            meas_yaw = -1*meas_yaw
            angles_man.append([meas_pitch, meas_roll, meas_yaw])
            #print(a, g, m)
            print("Pitch_BNO: {:.2f}\t Manhony: {:.2f}\tRoll_BNO: {:.2f}\t Manhony: {:.2f}\tYaw_BNO: {:.2f}\t Manhony: {:.2f}\t".format(pitch_bno, meas_pitch, roll_bno, meas_roll, yaw_bno, meas_yaw))
    prev_yaw = meas_yaw
    prev_roll = meas_roll
    prev_pitch = meas_pitch
    t = t + 1
    if t == epochs:
        break

angles_bno = np.array(angles_bno)
angles_man = np.array(angles_man)
#print(angles)
np.savetxt("comparision.csv", angles_bno, delimiter = ",")

## Calculating RMSE

rmse_p = np.sqrt(np.square(angles_bno[:,0] - angles_man[:,0]).mean())
rmse_r = np.sqrt(np.square(angles_bno[:,1] - angles_man[:,1]).mean())
rmse_y = (angles_bno[:,2] - angles_man[:,2] + 180) % (360) - 180
rmse_y = np.sqrt(np.square(rmse_y).mean())

print(rmse_p, rmse_r, rmse_y)


fig, axs = plt.subplots(3)
fig.suptitle("Euler angles")
axs[0].plot(angles_man[1:,0], label = "$\\theta_{Manhony}$")
axs[0].plot(angles_bno[1:,0], label = "$\\theta_{BNO055}$")
axs[0].grid()
axs[0].legend()
axs[0].set_ylabel("Pitch (degree)")
axs[1].plot(angles_man[1:,1], label = "$\phi_{Manhony}$")
axs[1].plot(angles_bno[1:,1], label = "$\phi_{BNO055}$")
axs[1].legend()
axs[1].set_ylabel("Roll (degree)")
axs[1].grid()
axs[2].plot(angles_man[1:,2], label = "$\psi_{Manhony}$")
axs[2].plot(angles_bno[1:,2], label = "$\psi_{BNO055}$")
axs[2].legend()
axs[2].set_ylabel("Yaw (degree)")

plt.grid()
plt.legend()
plt.savefig("latest_results.png")
plt.show()
