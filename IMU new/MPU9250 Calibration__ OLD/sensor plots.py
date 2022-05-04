import numpy as np
import matplotlib.pyplot as plt

## Initialize MPU
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(address_ak = AK8963_ADDRESS, address_mpu_master = MPU9050_ADDRESS_68, address_mpu_slave = None, bus = 1, gfs = GFS_1000, afs = AFS_8G, mfs = AK8963_BIT_16, mode = AK8963_MODE_C100HZ)
## Configure the registers and set calibrated biases 
mpu.configure()

accel = []
gyr = []
for t in range(1, 1000):
    ## Gives list
    a = mpu.readAccelerometerMaster()
    #a = [item*9.80665 for item in a]
    
    g = mpu.readGyroscopeMaster()
    #g = [item/57.2958 for item in g]
        
    m = mpu.readMagnetometerMaster()
    
    accel.append(a)
    gyr.append(g)
accel = np.array(accel)
gyr = np.array(gyr)


fig, axs = plt.subplots(3)
fig.suptitle("Accelerometer Readings")
axs[0].hist(accel[:,0], bins = 20)
axs[0].legend("x")
#axs[0].set_ylabel("$a_{x}$ g")
axs[1].hist(accel[:,1], bins = 20)
axs[1].legend("y")
#axs[1].set_ylabel("$a_{y}$ g")
axs[2].hist(accel[:,2], bins = 20)
axs[2].legend("z")
axs[2].set_xlabel("$a_{x}$, $a_{y}$, and $a_{z}$ in g.")


fig2, axs2 = plt.subplots(3)
fig2.suptitle("Gyroscope Readings")
axs2[0].hist(gyr[:,0], bins = 20)
axs2[0].legend("x")
axs2[1].hist(gyr[:,1], bins = 20)
axs2[1].legend("y")
axs2[2].hist(gyr[:,2], bins = 20)
axs2[2].legend("z")
axs2[2].set_xlabel("$g_{x}$, $g_{y}$, and $g_{z}$ in degree/sec.")


plt.legend()
plt.show()
