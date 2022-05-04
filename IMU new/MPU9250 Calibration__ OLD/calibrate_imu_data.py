import os
os.system("sudo pigpiod")

import time
import pigpio
time.sleep(1)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys

import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250


from quat2euler import quat2euler_angle

from ahrs.filters import Mahony
orientation = Mahony(k_P = 4, k_I = 1.5)



mpu = MPU9250(address_ak = AK8963_ADDRESS, address_mpu_master = MPU9050_ADDRESS_68, address_mpu_slave = None, bus = 1, gfs = GFS_1000, afs = AFS_8G, mfs = AK8963_BIT_16, mode = AK8963_MODE_C100HZ)

mpu.configure()
#mpu.calibrateAK8963()
#mpu.configure()


print("Abias {} Gbias {}, MagScale {}, Mbias{} ".format(mpu.abias, mpu.gbias, mpu.magScale, mpu.mbias))

print("Start moving the IMU")
time.sleep(1.5)

## Plotting magnetometer values before calibration
mag_data = []
for i in range(10000):
    mag_data.append(mpu.readMagnetometerMaster())

mag_data = np.array(mag_data)

fig = plt.figure()
ax = plt.axes(projection = "3d")
ax.scatter3D(mag_data[:,0], mag_data[:,1], mag_data[:,2])
ax.scatter3D(0,0,0, marker = "^")
ax.set_xlabel("Mx")
ax.set_ylabel("My")
ax.set_zlabel("Mz")
ax.set_title("Magnetometer Readings Before Calibration")




print("Calibrating in 1 sec, keep that IMU moving")
time.sleep(1.5)
calibration_epochs = 1
movement_flag = True

calibrated_magScale = []
calibrated_mbias = []

for i in range(calibration_epochs):
    if movement_flag == True:
        movement_flag == False
        print("Start IMU Movement")        
        mpu.calibrateAK8963()
        mpu.configure()
        calibrated_magScale.append(mpu.magScale)
        calibrated_mbias.append(mpu.mbias)
        print(mpu.magScale, mpu.mbias)
        inp = input("One cycle done, enter Y")
        if inp == "Y":
            movement_flag = True
        else:
            movement_flag == False

calibrated_magScale = np.array(calibrated_magScale)
calibrated_mbias = np.array(calibrated_mbias)

print("Mag Scale {}".format(np.mean(calibrated_magScale, axis = 0)))

print("Mag Bias {}".format(np.mean(calibrated_mbias, axis = 0)))

print("IMU calibration done, in 2 secs do IMU movement once again")
time.sleep(2)

mag_data = []
for i in range(10000):
    mag_data.append(mpu.readMagnetometerMaster())

mag_data = np.array(mag_data)

fig = plt.figure()
ax = plt.axes(projection = "3d")
ax.scatter3D(mag_data[:,0], mag_data[:,1], mag_data[:,2])
ax.set_xlabel("Mx")
ax.set_ylabel("My")
ax.set_zlabel("Mz")
ax.set_title("Magnetometer Readings after Calibration")
ax.scatter3D(0,0,0, marker = "^")
plt.show()

print("The finally calibrated values are MagScale {}, Mbias{} ".format(mpu.magScale, mpu.mbias))


        
