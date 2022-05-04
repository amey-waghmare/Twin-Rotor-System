import os
os.system("sudo pigpiod")

import time
time.sleep(1)


### Establish a connection so that wifi command center can send commands here
import socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5051
sock.setblocking(False)
sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
time.sleep(1)


## Tell here the pins where ESC are connected (These are gpio numbers and not board numbers)
ESC1 = 27
ESC2 = 17

### Initialize the ESC
from esc import *

print("Wait 3 sec")
esc1 = ESC(ESC1)
esc2 = ESC(ESC2)
time.sleep(3)
print("ESC Calibrated, Now working on IMU")


import numpy as np
import matplotlib.pyplot as plt
import math
from PID import *


dt_angles = 0.01

import sys

import time



from quat2euler import quat2euler_angle

from ahrs.filters import Mahony

## Mahony
#from mahony_ahrs import Mahony    ## Updated library with corrected Integrator Implementation
from ahrs import Quaternion
## Without disturbance
#orientation = Mahony(frequency = 100.0,k_P = 45, k_I = 30)
orientation = Mahony(frequency = 100.0,k_P = 7, k_I = 5) ### Working great for steady state




## Initialize MPU
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(address_ak = AK8963_ADDRESS, address_mpu_master = MPU9050_ADDRESS_68, address_mpu_slave = None, bus = 1, gfs = GFS_1000, afs = AFS_8G, mfs = AK8963_BIT_16, mode = AK8963_MODE_C100HZ)
## Configure the registers and set calibrated biases 
mpu.configure()
mpu.calibrateMPU6500()
### Values in Powai
mpu.magScale = [0.97976, 0.98861, 1.033333]
mpu.mbias = [-16.57176816, 42.9279313, -45.592276]

### Values in Nagpur
#mpu.magScale = [1.381642, 0.97435897, 1.02702703]
#mpu.mbias = [18.44951923, 13.66289396, -45.109660220]
#mag_cal_data = [11.865234375, 7.5439453125, -41.162109375]
mpu.configure()

print("Abias {} Gbias {}, MagScale {}, Mbias{} ".format(mpu.abias, mpu.gbias, mpu.magScale, mpu.mbias))
epochs = 15000
Q = np.tile([1.0, 0.0,0.0,0.0], (epochs,1))
eul = []

pwm_to_give = hover_pwm
esc1.set(pwm_to_give)
goal_pitch = 0

PID_PI_P_GAIN = 1
PID_PI_I_GAIN = 0.7
PID_PI_D_GAIN = 0.2

pa_pid = PID(PID_PI_P_GAIN, PID_PI_I_GAIN, PID_PI_D_GAIN)

err = []

angles = []
t = 0

### Required by socket ###
data = b'5'
while True:
    
    ## Gives list
    a = mpu.readAccelerometerMaster()
    a = [item*9.80665 for item in a]
    
    g = mpu.readGyroscopeMaster()
    g = [item/57.2958 for item in g]
    
    m = mpu.readMagnetometerMaster()
    #for ele in range(3):
    #    m[ele] = m[ele] - mag_cal_data[ele]
    #m = [item for item in m]    
        
    #Q[t] = orientation.updateMARG(Q[t-1], gyr = g, acc = a, mag = m)    
    Q[t] = orientation.updateIMU(Q[t-1], gyr = g, acc = a)        
    meas_pitch, meas_roll, meas_yaw = quat2euler_angle(Q[t,0], Q[t,1], Q[t,2], Q[t,3])

    
    ##### Now reading the goal angle ######
    try:
        data = sock.recv(1024, socket.MSG_DONTWAIT)
        print(data)
    except BlockingIOError as e:
        #print("passing")
        pass
    if not isinstance(data, str):
        data = data.decode("utf-8")
    goal_pitch = int(data[-2:]) - 35
    
    
    p_out, i_out, d_out = pa_pid.Compute(meas_pitch, goal_pitch, dt_angles)
    error = p_out + i_out + d_out
    err.append(error)
    
    gravity_to_compensate = int(round(14*9.81*np.sin(np.radians(goal_pitch))))
    pwm_to_give = hover_pwm + gravity_to_compensate + int(round(error))
    esc1.set(pwm_to_give)
    #time.sleep(0.1)
    
    ## Debug
    #print("PWM: {:.2f}\t error:{:.2f} PITCH: {:.2f}\t ROLL: {:.2f}\t YAW: {:.2f}".format(pwm_to_give, error, X, Y, Z))
    print("t: {}\t PWM: {}\tERR: {:.2f}\tPITCH: {:.2f}\t YAW: {:.2f}\t Gravity:{:.2f}".format(t, pwm_to_give,error, meas_pitch, meas_yaw, gravity_to_compensate))
    angles.append([meas_pitch, meas_roll, meas_yaw])
    t = t + 1
    if t == epochs:
        break

angles = np.array(angles)
np.savetxt("angles__1.csv", angles, delimiter = ",")

fig, axs = plt.subplots(3)
fig.suptitle("Euler angles")
axs[0].plot(angles[1000:,0])
axs[0].grid()
axs[0].legend("Pitch")
axs[0].set_ylabel("Pitch (degree)")
axs[1].plot(angles[1000:,1])
axs[1].legend("Roll")
axs[1].set_ylabel("Roll (degree)")
axs[1].grid()
axs[2].plot(angles[1000:,2])
axs[2].legend("Yaw")
axs[2].set_ylabel("Yaw (degree)")

plt.grid()
plt.legend()
plt.savefig("latest_results.png")
plt.show()

esc1.kill_esc()
esc2.kill_esc()
pi.stop()
