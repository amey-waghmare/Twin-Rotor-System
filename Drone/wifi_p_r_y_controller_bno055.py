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


goal_pitch = 0
goal_roll = 0
goal_yaw = 0

pitch_P_GAIN = 1.5
pitch_I_GAIN = 2.0
pitch_D_GAIN = 1.2

roll_P_GAIN = 0.5
roll_I_GAIN = 0.0
roll_D_GAIN = 0.0

yaw_P_GAIN = 0.8
yaw_I_GAIN = 2.0
yaw_D_GAIN = 1.2

dt_angles = 0.01

pitch_pid = PID(pitch_P_GAIN, pitch_I_GAIN, pitch_D_GAIN)
roll_pid = PID(roll_P_GAIN, roll_I_GAIN, roll_D_GAIN)
yaw_pid = YAW_PID(yaw_P_GAIN, yaw_I_GAIN, yaw_D_GAIN)



goal_angles = []
angles = []


### Required by socket ###

prev_pitch = 0
prev_roll = 0
prev_yaw = 0

## For calculating errors


data = b'0'
while True:
    
    ##### Now reading the goal angle ######
    try:
        data = sock.recv(1024, socket.MSG_DONTWAIT)
        print(data)
    except BlockingIOError as e:
        #print("passing")
        pass
    if not isinstance(data, str):
        data = data.decode("utf-8")
        
    
    if data[:3] == "pit":
        goal_pitch = int(data[-3:]) - 35
        ## debug
        #print("goal_pitch is {}".format(goal_pitch))
    
    elif data[:3] == "yaw":
        goal_yaw = int(data[-3:])
        
    elif data[:3] == "rol":
        goal_roll = int(data[-3:]) - 35
    
    elif data == "alloff":
        break
    
    ### Read Sensor data
    meas_yaw, meas_roll, meas_pitch = sensor.euler
    #Q = sensor.quaternion
    #meas_pitch, meas_roll, meas_yaw = quat2euler_angle(Q[0], Q[1], Q[2], Q[3])
    
    
    
    if meas_yaw != None or meas_roll != None or meas_pitch != None:
        ## Pitch PID controller
        if meas_pitch >=360 or meas_pitch <= -360:
            meas_pitch = prev_pitch
        if meas_roll >=360 or meas_roll <= -360:
            meas_roll = prev_roll
        if meas_yaw > 365 or meas_yaw < -2:
            meas_yaw = prev_yaw
        
        ## Pitch PID
        pitch_p_out, pitch_i_out, pitch_d_out = pitch_pid.Compute(meas_pitch, goal_pitch, dt_angles)
        pitch_error = pitch_p_out + pitch_i_out + pitch_d_out
        
    
        gravity_to_compensate = int(round(30*9.81*np.sin(np.radians(goal_pitch))))
        pitch_pwm_to_give = hover_pwm + gravity_to_compensate + int(round(pitch_error))
        
##TODO        ## Set pitch motors here
        #esc1.set(pitch_pwm_to_give)
        #time.sleep(0.1)
        
        ## Roll PID
        roll_p_out, roll_i_out, roll_d_out = roll_pid.Compute(meas_roll, goal_roll, dt_angles)
        roll_error = roll_p_out + roll_i_out + roll_d_out
##TODO        ## Set yaw motors here
        
        
        
        ## Yaw PID controller
        #yaw_error = yaw_pid.Error(meas_yaw, goal_yaw)
        yaw_p_out, yaw_i_out, yaw_d_out = yaw_pid.Compute(meas_yaw, goal_yaw, dt_angles)
        yaw_error = yaw_p_out + yaw_i_out + yaw_d_out
        
        
        
        if yaw_error >= 0:
            yawL_pwm_to_give = 1260 + int(yaw_error)
            yawR_pwm_to_give = 0
            
        elif goal_yaw - meas_yaw < 0:
            yawR_pwm_to_give = 1270 + abs(int(yaw_error))
            yawL_pwm_to_give = 0
            
        esc2.set(yawL_pwm_to_give)
        esc3.set(yawR_pwm_to_give)
        
        
        #print("YAW ANGLE: {:.2f}\tYaw Error: {:.2f}\tL motor: {}\t R Motor: {}".format(meas_yaw, yaw_error, yawL_pwm_to_give,yawR_pwm_to_give))
        
        prev_yaw = meas_yaw
        prev_roll = meas_roll
        prev_pitch = meas_pitch
        
        
        ## Debug
        print("PITCH: {:.2f}\t ROLL: {:.2f}\t YAW: {:.2f}".format( meas_pitch, meas_roll, meas_yaw))
        #print("PITCH_ERR: {:.2f}\tGOAL_PITCH {:.2f}\tPITCH: {:.2f}\t GOAL_YAW {:.2f}\tYAW: {:.2f}\t Gravity:{:.2f}".format(pitch_error, goal_pitch, meas_pitch, goal_yaw, meas_yaw, gravity_to_compensate))
        #print("YAW: {:.2f}\tGOAL_YAW: {:.2f}\tL PWM: {}\tR PWM: {}\t".format(meas_yaw, goal_yaw, yawL_pwm_to_give, yawR_pwm_to_give))
        angles.append([meas_pitch, meas_roll, meas_yaw])
        goal_angles.append([goal_pitch, goal_roll, goal_yaw])
    


angles = np.array(angles)
goal_angles = np.array(goal_angles)
np.savetxt("angles__1.csv", angles, delimiter = ",")

esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()

## Calculating RMSE

rmse_p = np.sqrt(np.square(angles[:,0] - goal_angles[:,0]).mean())
rmse_r = np.sqrt(np.square(angles[:,1] - goal_angles[:,1]).mean())
rmse_y = (angles[:,2] - goal_angles[:,2] + 180) % (360) - 180
rmse_y = np.sqrt(np.square(rmse_y).mean())

print("RMSE P {}\t RMSE R {}\t RMSE Y {}".format(rmse_p,rmse_r, rmse_y))

fig, axs = plt.subplots(3)
fig.suptitle("Euler angles")
axs[0].plot(angles[1000:,0])
axs[0].plot(goal_angles[1000:,0])
axs[0].grid()
axs[0].legend("Pitch")
axs[0].set_ylabel("Pitch (degree)")
axs[1].plot(angles[1000:,1])
axs[1].plot(goal_angles[1000:,1])
axs[1].legend("Roll")
axs[1].set_ylabel("Roll (degree)")
axs[1].grid()
axs[2].plot(angles[1000:,2])
axs[2].plot(goal_angles[1000:,2])
axs[2].legend("Yaw")
axs[2].set_ylabel("Yaw (degree)")

plt.grid()
plt.legend()
plt.savefig("latest_results.png")
plt.show()


