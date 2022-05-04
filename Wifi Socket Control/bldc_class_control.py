## This is the wifi code

import os
os.system("sudo pigpiod")

import time
import pigpio
time.sleep(1)



### Establish a connection so that wifi command center can send commands here
import socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5051
sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
time.sleep(1)


## Tell here the pins where ESC are connected (These are gpio numbers and not board numbers)
ESC1 = 17
ESC2 = 27
ESC3 = 22


pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC1, 0)

max_value = 1700
min_value = 1199


class ESC:
    def __init__(self, pin):#, location)#, rotation):
        self.gpio_pin = pin
        
        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        # In other words, ARM
        #-------------------------------------------------------------------------------------------
        self.pulse_width = 0
        pi.set_servo_pulsewidth(self.gpio_pin, 0)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(self.gpio_pin, max_value)
        time.sleep(0.5)        
        pi.set_servo_pulsewidth(self.gpio_pin, min_value)
        time.sleep(0.5)

    def set(self, pulse_width):
        pulse_width = pulse_width if pulse_width >= min_value else min_value
        pulse_width = pulse_width if pulse_width <= max_value else max_value

        self.pulse_width = pulse_width

        pi.set_servo_pulsewidth(self.gpio_pin, self.pulse_width)
        
        ### This is to debug        
        print("t {}\tpulse width :{}".format(cnt,self.pulse_width))

    def kill_esc(self):
        pi.set_servo_pulsewidth(self.gpio_pin, 0)


print("Wait 3 sec")
esc1 = ESC(ESC1)
esc2 = ESC(ESC2)
esc3 = ESC(ESC3)
time.sleep(3)
print("ESC Calibrated, Now start commands")

esc1_pwm = 1300
esc2_pwm = 1300
esc3_pwm = 1300

step_change = 5

cnt = 0
while True:
    cnt = cnt + 1
    sock.settimeout(0.01)
    try:
        data, addr = sock.recvfrom(1024)
    except:
        data = None
    if data is None:
        continue
    ## The above data comes in bytes, need to convert it into string
    data = data.decode("utf-8")
    print(data, cnt)

    if data == "device1on":
        esc1.set(esc1_pwm)
    elif data == "device1off":
        esc1.set(min_value)
    elif data == "device2on":
        esc2.set(esc2_pwm)
    elif data == "device2off":
        esc2.set(min_value)
    elif data == "device3on":
        esc3.set(esc3_pwm)
    elif data == "device3off":
        esc3.set(min_value)
    
    elif data == "d1a":
        esc1_pwm = 1700 if esc1_pwm >= 1700 else esc1_pwm + step_change
        esc1.set(esc1_pwm)
    elif data == "d1s":
        esc1_pwm = 1199 if esc1_pwm <= 1199 else esc1_pwm - step_change
        esc1.set(esc1_pwm)
    elif data == "d2a":
        esc2_pwm = 1700 if esc2_pwm >=1700 else esc2_pwm + step_change
        esc2.set(esc2_pwm)
    elif data == "d2s":
        esc2_pwm = 1199 if esc2_pwm <= 1199 else esc2_pwm - step_change
        esc2.set(esc2_pwm)
    elif data == "d3a":
        esc3_pwm = 1700 if esc3_pwm >=1700 else esc3_pwm + step_change
        esc3.set(esc3_pwm)
    elif data == "d3s":
        esc3_pwm = 1199 if esc3_pwm <= 1199 else esc3_pwm - step_change
        esc3.set(esc3_pwm)     
    
    
    elif data == "alloff":
        esc1.kill_esc()
        esc2.kill_esc()
        esc3.kill_esc()
        pi.stop()
        break    













################################################################################
################################################################################
## The below set of code is just standard execution  ##


#print("30 sec on")
#esc1.set(1500)
#esc2.set(1500)
#time.sleep(30)

#print("5 sec off")
#esc1.set(min_value)
#esc2.set(min_value)
#time.sleep(5)

#print("20 sec on")
#esc1.set(1300)
#esc2.set(1300)
#time.sleep(20)

#print("2 sec off")
#esc1.set(min_value)
#esc2.set(min_value)
#time.sleep(2)

#esc1.kill_esc()
#esc2.kill_esc()
#pi.stop()



## I was here
