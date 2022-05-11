import os
os.system("sudo pigpiod")

import time
time.sleep(1)


### Establish a connection so that wifi command center can send commands here
import socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5050
sock.setblocking(False)
sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
time.sleep(1)


## Tell here the pins where ESC are connected (These are gpio numbers and not board numbers)
ESC1 = 17
ESC2 = 27
ESC3 = 22

### Initialize the ESC
from esc import *

print("Wait 3 sec")
esc1 = ESC(ESC1)
esc2 = ESC(ESC2)
esc3 = ESC(ESC3)
time.sleep(3)


#### IMU STUFF Adafruit BNO055
import board
import adafruit_bno055

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("ESC Calibrated, Now working on IMU, Make some movement for IMU!!!")

for i in range(2000):
    print(sensor.calibration_status)
print("calibration done!!!")


### If using Quaternions
from quat2euler import quat2euler_angle


### Now use sensor.euler to get values

import numpy as np
import matplotlib.pyplot as plt
import math
from PID import *

import sys

pitch_pwm_to_give = hover_pwm
esc1.set(pitch_pwm_to_give)

yawL_pwm_to_give = hover_pwm
esc2.set(yawL_pwm_to_give)

yawR_pwm_to_give = hover_pwm
esc3.set(yawR_pwm_to_give)


goal_pitch_rad = 0.35
goal_roll_rad = 0.00
goal_yaw_rad = 0.00

dt_angles = 0.01


goal_angles = []
angles = []

gyro_data = []

uk_data = []

t = 0


##########################################################################################################################################
######    LQR    #######

#10x45
#A = np.array([[1, 0, 0.009985, 0],[-1.503e-6, 1, -5.012e-9, 0.009936],[-0.008234, 0, 0.997, 0],[-0.0003, 0, -1.502e-6, 0.9872]])
#B = np.array([[7.397e-9, -4.615e-11],[-2.461e-10, 1.378e-9],[1.479e-6, -9.225e-9],[-4.911e-8, 2.75e-7]])
### LQR gain
#K = np.array([[0.5204e4, -1.6410e4, 0.6346e4, -1.1363e4],[-0.0371e4, 1.8694e4, 0.0028e4, 1.2574e4]])

#9x4.3

   
A = np.array([[1, 0, 0.01, 0],[-1.032e-07, 1, -3.44e-10, 0.009999],[0.0126, 0, 1, 0],[-2.064e-05, 0, -1.032e-07, 0.9997]])
B = np.array([[6.292e-09, -3.722e-13],[-9.408e-12, 2.489e-10],[1.258e-06, -7.444e-11],[-1.881e-09, 4.978e-08]])
### LQR gain   
K = np.array([[1.991048898122167e4, -0.000000345763822e4, 1.772113020821166e4, -0.000013548546424e4],[-0.000119013390064e4, 0.000099999290716e4, -0.000099521388586e4, 0.003917872027825e4]])

 
  

#epochs = 8000

### Required by socket ###

prev_pitch = 0
prev_roll = 0
prev_yaw = 0

prev_gyro_x = 0
prev_gyro_z = 0


data = b'0'
while True:
    
    ########## Now reading the goal angle #############
    ###################################################
    try:
        data = sock.recv(1024, socket.MSG_DONTWAIT)
        print(data)
    
    except BlockingIOError as e:
        data = "qwerty"
        #uncomment this if using App slider
        #print("passing")
        #pass
    if not isinstance(data, str):
        data = data.decode("utf-8")
    ####################################################   
    
    if data == "pitrada":
        goal_pitch_rad = goal_pitch_rad + 0.01 
    elif data == "pitradz":
        goal_pitch_rad = goal_pitch_rad - 0.01
    
    elif data == "yawrada":
        goal_yaw_rad = goal_yaw_rad + 0.01 
    elif data == "yawradz":
        goal_yaw_rad = goal_yaw_rad - 0.01
    
    elif data == "alloff":
        break
    
    
    
    goal_angles.append([goal_pitch_rad, goal_yaw_rad])
    
    
    
    
    
    ### Read Sensor data
    meas_yaw, meas_roll, meas_pitch = sensor.euler
    meas_gyro_x, meas_gyro_y, meas_gyro_z = sensor.gyro
    #Q = sensor.quaternion
    #meas_pitch, meas_roll, meas_yaw = quat2euler_angle(Q[0], Q[1], Q[2], Q[3])
    
    
    
    if meas_yaw != None and meas_roll != None and meas_pitch != None and meas_gyro_x != None and meas_gyro_z != None:
        # remove spikes
        if meas_pitch >=360 or meas_pitch <= -360:
            meas_pitch = prev_pitch
        if meas_yaw > 365 or meas_yaw < -2:
            meas_yaw = prev_yaw
        if meas_gyro_x > 10 or meas_gyro_x < -10:
            meas_gyro_x = prev_gyro_x
        if meas_gyro_z > 10 or meas_gyro_z < -10:
            meas_gyro_z = prev_gyro_z
            
        
        ### Convert angles from degrees to radians
        meas_pitch_rad = np.radians(meas_pitch)
        meas_yaw_rad = np.radians(meas_yaw)
        
        ## desired states
        xd = np.array([[goal_pitch_rad - (0.3)],[goal_yaw_rad - (0)],[0 - (0)],[0 - (0)]])
        
        us = np.dot(np.dot(np.dot(np.linalg.inv(np.dot(B.transpose(),B)), B.transpose()), A ), xd)
        del_x = np.array([[meas_pitch_rad - (0.3)],[meas_yaw_rad - (0)],[meas_gyro_x - (0)],[meas_gyro_z - (0)]])
        
        
        uk = us - np.dot(K, (del_x - xd))
        
        uk = uk + np.array([[2.789216555670474e+03], [1.103499533350318e+02]])
        
        
        ## Bound input values
        if uk[0] > 3500:
            uk[0] =3500
        elif uk[0] < -3500:
            uk[0] = -3500
        
            
        if uk[1] > 2000:
            uk[1] = 2000
        elif uk[1] < -2000:
            uk[1] = -2000
        
        #uk[0] = abs(uk[0])
        
        ### Omega to PWM
        weight_pwm = np.array([1.26640802e3, -2.30569152e-2, 3.30102447e-5])
        
        
        pwm_m = int(np.dot(np.array( [1, uk[0], np.power(uk[0], 2)] ), weight_pwm))
        pwm_t = int(np.dot(np.array( [1, uk[1], np.power(uk[1], 2)] ), weight_pwm))
        #pwm_t = 1200
        
        pwm_m = pwm_m if pwm_m >= 1000 else 1000
        pwm_m = pwm_m if pwm_m <= 1650 else 1650
        
        if pwm_t >= 0:
            pwm_t_L = int(pwm_t)
            pwm_t_R = 0
            
        elif pwm_t < 0:
            pwm_t_L = 0
            pwm_t_R = int(pwm_t)
        
        esc1.set(pwm_m)
        esc2.set(pwm_t_L)
        esc3.set(pwm_t_R)
        
        ## for spikes
        prev_yaw = meas_yaw
        prev_pitch = meas_pitch
        prev_gyro_x = meas_gyro_x
        prev_gyro_z = meas_gyro_z
        
        ## Debug
        print("{}\t{}\t{}\t{}\tgola_pitch: {:.2f}\tgoal_yaw: {:.2f}\tPITCH: {:.2f}\t ROLL: {:.2f}\t YAW: {:.2f}\tP_Rate: {:.2f}\tY_Rate:{:.2f}".format(int(uk[0]), int(uk[1]), pwm_m, pwm_t, goal_pitch_rad, goal_yaw_rad ,meas_pitch, meas_roll, meas_yaw, meas_gyro_x ,meas_gyro_z))
        
        uk_data.append([uk[0], uk[1]])
        
        angles.append([meas_pitch_rad, meas_roll, meas_yaw_rad])
        
        gyro_data.append([meas_gyro_x, meas_gyro_z])
    
    ### Break after certain  epochs
    #t = t + 1
    #if t == epochs:
    #    break

angles = np.array(angles)
goal_angles = np.array(goal_angles)
gyro_data = np.array(gyro_data)
uk_data = np.array(uk_data)

#np.savetxt("angles__1.csv", angles, delimiter = ",")

esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()




fig, axs = plt.subplots(6)
fig.suptitle("States")

axs[0].plot(angles[1:,0])
axs[0].plot(goal_angles[1:,0])
axs[0].legend("Pitch")
axs[0].set_ylabel("Pitch (degree)")
axs[0].grid()

axs[1].plot(angles[1:,2])
axs[1].plot(goal_angles[1:,1])
axs[1].legend("Yaw")
axs[1].set_ylabel("Yaw (degree)")
axs[1].grid()

#axs[2].plot(angles[1000:,0])
axs[2].plot(gyro_data[1:,0])
axs[2].legend("Pitch Rate")
axs[2].set_ylabel("Pitch Rate rad/sec")
axs[2].grid()

#axs[2].plot(angles[1000:,1])
axs[3].plot(gyro_data[1:,1])
axs[3].legend("Yaw Rate")
axs[3].set_ylabel("Yaw Rate rad/sec")
axs[3].grid()

axs[4].plot(uk_data[1:,0])
axs[4].legend("u1")
axs[4].set_ylabel("U1")
#axs[4].set_ylim([0,4000])
axs[4].grid()

axs[5].plot(uk_data[1:,1])
axs[5].legend("u2")
axs[5].set_ylabel("U2")
#axs[5].grid()


plt.grid()
plt.legend()
plt.savefig("latest_results.png")
plt.show()


