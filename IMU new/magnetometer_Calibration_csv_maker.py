import os
os.system("sudo pigpiod")

import time
import pigpio
time.sleep(1)



### Establish a connection so that wifi command center can send commands here
#import socket 
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#UDP_IP = "0.0.0.0"
#UDP_PORT = 5050
#sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
#time.sleep(1)


## Tell here the pins where ESC are connected (These are gpio numbers and not board numbers)
#ESC1 = 27
#ESC2 = 17


#pi = pigpio.pi()
#pi.set_servo_pulsewidth(ESC1, 0)

#max_value = 1700
#min_value = 1199

#hover_pwm = 1500

#class ESC:
#    def __init__(self, pin):#, location)#, rotation):
#        self.gpio_pin = pin
#        
#        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        # In other words, ARM
        #-------------------------------------------------------------------------------------------
#        self.pulse_width = 0
#        pi.set_servo_pulsewidth(self.gpio_pin, 0)
#        time.sleep(0.5)
#        pi.set_servo_pulsewidth(self.gpio_pin, max_value)
#        time.sleep(0.5)        
#        pi.set_servo_pulsewidth(self.gpio_pin, min_value)
#        time.sleep(0.5)

#    def set(self, pulse_width):
#        pulse_width = pulse_width if pulse_width >= min_value else min_value
#        pulse_width = pulse_width if pulse_width <= max_value else max_value

#        self.pulse_width = pulse_width

#        pi.set_servo_pulsewidth(self.gpio_pin, self.pulse_width)
        
        ### This is to debug        
        #print("pulse width :{}".format(self.pulse_width))

#    def kill_esc(self):
#        pi.set_servo_pulsewidth(self.gpio_pin, 0)


#print("Wait 3 sec")
#esc1 = ESC(ESC1)
#esc2 = ESC(ESC2)
#time.sleep(3)
#print("ESC Activated, Now working on IMU")


import numpy as np
import matplotlib.pyplot as plt
#from PID import *

#PID_PI_P_GAIN = 2.0
#PID_PI_I_GAIN = 0.0
#PID_PI_D_GAIN = 0.0

#dt_angles = 0.01

import sys

import time



#from quat2euler import quat2euler_angle

## Mahony
#from ahrs.filters import Mahony
#from ahrs import Quaternion
#orientation = Mahony(frequency = 100.0, k_P = 7, k_I = 3)


## Initialize MPU
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(address_ak = AK8963_ADDRESS, address_mpu_master = MPU9050_ADDRESS_68, address_mpu_slave = None, bus = 1, gfs = GFS_1000, afs = AFS_8G, mfs = AK8963_BIT_16, mode = AK8963_MODE_C100HZ)
## Configure the registers and set calibrated biases 
mpu.configure()
mpu.calibrateMPU6500()
#mpu.magScale = [0.9910514541387024, 0.9736263736263736, 1.0374707259953162]
#mpu.mbias = [-5.534855769230769, 37.99165998931623, -46.97159741300366]

mpu.configure()

print("Abias {} \n Gbias {}\n MagScale {}\n Mbias{}\n\n\n".format(mpu.abias, mpu.gbias, mpu.magScale, mpu.mbias))
epochs = 30000
magneto = []
#Q = np.tile([1.0, 0.0,0.0,0.0], (epochs,1))
#eul = []


#pa_pid = PID(PID_PI_P_GAIN, PID_PI_I_GAIN, PID_PI_D_GAIN)
#angles = []
#pwm_pitch = []


for t in range(1, epochs):
    ## Gives list
    a = mpu.readAccelerometerMaster()
    a = [item*9.80665 for item in a]
    
    g = mpu.readGyroscopeMaster()
    g = [item/57.2958 for item in g]
        
    m = mpu.readMagnetometerMaster()
    #m = [item for item in m]
    magneto.append(m)
    print(m)
    
    
#    Q[t] = orientation.updateMARG(Q[t-1], gyr = g, acc = a, mag = m)    
    #Q[t] = orientation.updateIMU(Q[t-1], gyr = g, acc = a)        
#    X,Y,Z = quat2euler_angle(Q[t,0], Q[t,1], Q[t,2], Q[t,3])
        
        
#    p_out, i_out, d_out = pa_pid.Compute(X, 0, dt_angles)
#    error = p_out + i_out + d_out
    #print(error)    
#    pwm_to_give = int(round(hover_pwm + error))
#    esc1.set(pwm_to_give)    
    #time.sleep(0.1)
#    pwm_pitch.append([pwm_to_give,X])
    ## Debug
    #print("PWM: {:.2f}\t error:{:.2f} PITCH: {:.2f}\t ROLL: {:.2f}\t YAW: {:.2f}".format(pwm_to_give, error, X, Y, Z))
    #print("PITCH: {:.2f} \t ROLL: {:.2f} \t YAW: {:.2f} ".format(X, Y, Z))
#    print("PWM: {:.2f}\t PITCH: {:.2f} \t ROLL: {:.2f} \t YAW: {:.2f}".format(pwm_to_give, X,Y,Z))    
#    angles.append([X,Y,Z])

magneto = np.array(magneto)
np.savetxt("magnetometer_readings.csv", magneto, delimiter = "\t")


#angles = np.array(angles)

#fig, axs = plt.subplots(3)
#fig.suptitle("Euler angles")
#axs[0].plot(angles[:,0])
#axs[0].grid()
#axs[0].legend("Pitch")
#axs[0].set_ylabel("Pitch (degree)")
#axs[1].plot(angles[:,1])
#axs[1].legend("Roll")
#axs[1].set_ylabel("Roll (degree)")
#axs[1].grid()
#axs[2].plot(angles[:,2])
#axs[2].legend("Yaw")
#axs[2].set_ylabel("Yaw (degree)")

#pwm_pitch = np.array(pwm_pitch)
#fig2 = plt.figure()
#fig2.suptitle("Pitch angle control")
#ax1 = fig2.add_subplot(211)
#ax1.plot(pwm_pitch[:,0])
#ax1.grid()
#ax1.set_label("PWM")
#ax1.set_ylabel("PWM")
#ax2 = fig2.add_subplot(212)
#ax2.plot(pwm_pitch[:,1])
#ax2.grid()
#ax2.set_label("$\phi$")
#ax2.set_ylabel("$\phi$")




#plt.legend()
#plt.show()

#esc1.kill_esc()
#esc2.kill_esc()
#pi.stop()
        
