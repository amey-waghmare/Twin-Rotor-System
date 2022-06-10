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
#esc2.set(yawL_pwm_to_give)

yawR_pwm_to_give = hover_pwm
#esc3.set(yawR_pwm_to_give)


goal_pitch = 13
goal_roll = 0
goal_yaw = 0

pitch_P_GAIN = 1.5
pitch_I_GAIN = 2.0
pitch_D_GAIN = 1.2

yaw_P_GAIN = 0.8
yaw_I_GAIN = 2.5
yaw_D_GAIN = 1.2

dt_angles = 0.01

pitch_pid = PID(pitch_P_GAIN, pitch_I_GAIN, pitch_D_GAIN)
yaw_pid = YAW_PID(yaw_P_GAIN, yaw_I_GAIN, yaw_D_GAIN)


pitch_err = []
yaw_err = []

goal_angles = []
io_data = []
t = 0

epochs = 8000

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
        
    if data == "alloff":
        break
    
    ### Read Sensor data
    meas_yaw, meas_roll, meas_pitch = sensor.euler
    #Q = sensor.quaternion
    #meas_pitch, meas_roll, meas_yaw = quat2euler_angle(Q[0], Q[1], Q[2], Q[3])
    
    
    
    if meas_yaw != None or meas_roll != None or meas_pitch != None:
        ## Pitch PID controller
        if meas_pitch >=360 or meas_pitch <= -360:
            meas_pitch = prev_pitch
        if meas_yaw > 365 or meas_yaw < -2:
            meas_yaw = prev_yaw
        
        #### Create PRBS Data Here
        u1 = round(30*np.sign(np.random.randn()))
        
        
        pitch_pwm_to_give = 1550 + u1
        esc1.set(pitch_pwm_to_give)
        print(pitch_pwm_to_give, meas_pitch)
        time.sleep(0.1)
        
        ## Yaw PID controller
        #yaw_error = yaw_pid.Error(meas_yaw, goal_yaw)
        yaw_p_out, yaw_i_out, yaw_d_out = yaw_pid.Compute(meas_yaw, goal_yaw, dt_angles)
        yaw_error = yaw_p_out + yaw_i_out + yaw_d_out
        #yaw_err.append(yaw_error)
        
        
        if yaw_error >= 0:
            yawL_pwm_to_give = 1262 + int(yaw_error)
            yawR_pwm_to_give = 0
            
        elif goal_yaw - meas_yaw < 0:
            yawR_pwm_to_give = 1252 + abs(int(yaw_error))
            yawL_pwm_to_give = 0
            
        #esc2.set(yawL_pwm_to_give)
        #esc3.set(yawR_pwm_to_give)
        
        
        #print("YAW ANGLE: {:.2f}\tYaw Error: {:.2f}\tL motor: {}\t R Motor: {}".format(meas_yaw, yaw_error, yawL_pwm_to_give,yawR_pwm_to_give))
        
        prev_yaw = meas_yaw
        prev_pitch = meas_pitch
        
        
        ## Debug
        #print("PITCH: {:.2f}\t ROLL: {:.2f}\t YAW: {:.2f}".format( meas_pitch, meas_roll, meas_yaw))
        #print("PITCH_ERR: {:.2f}\tGOAL_PITCH {:.2f}\tPITCH: {:.2f}\t GOAL_YAW {:.2f}\tYAW: {:.2f}\t Gravity:{:.2f}".format(pitch_error, goal_pitch, meas_pitch, goal_yaw, meas_yaw, gravity_to_compensate))
        #print("YAW: {:.2f}\tGOAL_YAW: {:.2f}\tL PWM: {}\tR PWM: {}\t".format(meas_yaw, goal_yaw, yawL_pwm_to_give, yawR_pwm_to_give))
        io_data.append([int(u1), int(yaw_error),meas_pitch, meas_yaw])
        goal_angles.append([goal_pitch, goal_roll, goal_yaw])
    
    ### Break after certain  epochs
    #t = t + 1
    #if t == epochs:
    #    break

io_data = np.array(io_data)
goal_angles = np.array(goal_angles)
np.savetxt("io_data.csv", io_data, delimiter = ",")

esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()

## Calculating RMSE

rmse_p = np.sqrt(np.square(io_data[:,2] - goal_angles[:,0]).mean())
rmse_y = (io_data[:,3] - goal_angles[:,2] + 180) % (360) - 180
rmse_y = np.sqrt(np.square(rmse_y).mean())

print("RMSE P {}\t RMSE Y {}".format(rmse_p, rmse_y))

fig, axs = plt.subplots(4)
fig.suptitle("Euler angles")
axs[0].plot(io_data[1:,2])
axs[0].plot(goal_angles[1:,0])
axs[0].grid()
axs[0].legend("Pitch")
axs[0].set_ylabel("Pitch (degree)")
#axs[1].plot(angles[1000:,1])
#axs[1].plot(goal_angles[1000:,1])
#axs[1].legend("Roll")
#axs[1].set_ylabel("Roll (degree)")
#axs[1].grid()
axs[1].plot(io_data[1:,3])
axs[1].plot(goal_angles[1:,2])
axs[1].legend("Yaw")
axs[1].set_ylabel("Yaw (degree)")
axs[2].plot(io_data[1:,0])
axs[2].legend("U1")
axs[2].set_ylabel("U1")



plt.grid()
plt.legend()
plt.savefig("latest_results.png")
plt.show()


