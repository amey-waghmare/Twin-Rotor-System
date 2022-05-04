import os
os.system("sudo pigpiod")

import time
time.sleep(1)


### Establish a connection so that wifi command center can send commands here
import socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5050
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

## Mahony
from mahony_ahrs import Mahony    ## Updated library with corrected Integrator Implementation
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
mpu.magScale = [1.0843706777316735, 0.9042675893886966, 1.0288713910761154]
mpu.mbias = [-5.534855769230769, 37.99165998931623, -46.97159741300366]

### Values in Nagpur
#mpu.magScale = [1.381642, 0.97435897, 1.02702703]
#mpu.mbias = [18.44951923, 13.66289396, -45.109660220]

mpu.configure()

print("Abias {} Gbias {}, MagScale {}, Mbias{} ".format(mpu.abias, mpu.gbias, mpu.magScale, mpu.mbias))
epochs = 8000
Q = np.tile([1.0, 0.0,0.0,0.0], (epochs,1))
eul = []

pwm_to_give = hover_pwm
esc1.set(pwm_to_give)
goal_pitch = 20

PID_PI_P_GAIN = 1
PID_PI_I_GAIN = 0.7
PID_PI_D_GAIN = 0.2

pa_pid = PID(PID_PI_P_GAIN, PID_PI_I_GAIN, PID_PI_D_GAIN)

err = []

angles = []

for t in range(1, epochs):
    ## Gives list
    a = mpu.readAccelerometerMaster()
    a = [item*9.80665 for item in a]
    
    g = mpu.readGyroscopeMaster()
    g = [item/57.2958 for item in g]
    
    m = mpu.readMagnetometerMaster()
    #m = [item for item in m]    
        
    Q[t] = orientation.updateMARG(Q[t-1], gyr = g, acc = a, mag = m)    
    #Q[t] = orientation.updateIMU(Q[t-1], gyr = g, acc = a)        
    meas_pitch, meas_roll, meas_yaw = quat2euler_angle(Q[t,0], Q[t,1], Q[t,2], Q[t,3])
    
        
    p_out, i_out, d_out = pa_pid.Compute(meas_pitch, goal_pitch, dt_angles)
    error = p_out + i_out + d_out
    err.append(error)
    
    gravity_to_compensate = int(round(14*9.81*np.sin(np.radians(goal_pitch))))
    pwm_to_give = hover_pwm + gravity_to_compensate + int(round(error))
    esc1.set(pwm_to_give)
    #time.sleep(0.1)
    
    ## Debug
    #print("PWM: {:.2f}\t error:{:.2f} PITCH: {:.2f}\t ROLL: {:.2f}\t YAW: {:.2f}".format(pwm_to_give, error, X, Y, Z))
    print("Frequency: {}\tPWM: {}\tERR: {:.2f}\tPITCH: {:.2f}\t Gravity:{:.2f}".format(pi.get_PWM_frequency(27), pwm_to_give,error, meas_pitch, gravity_to_compensate))
    angles.append([meas_pitch, meas_roll, meas_yaw])

angles = np.array(angles)

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
