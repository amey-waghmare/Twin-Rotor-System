class ESC:
    def __init__(self, pin):#, location)#, rotation):
        self.gpio_pin = pin
        
        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        # In other words, ARM
        #-------------------------------------------------------------------------------------------
        self.pulse_width = 0
        #pi.set_servo_pulsewidth(self.gpio_pin, 1000)
        #time.sleep(1)
        #pi.set_servo_pulsewidth(self.gpio_pin, 1050)
        #time.sleep(1)        
        pi.set_servo_pulsewidth(self.gpio_pin, min_value)
        time.sleep(1)

    def set(self, pulse_width):
        pulse_width = pulse_width if pulse_width >= min_value else min_value
        pulse_width = pulse_width if pulse_width <= max_value else max_value

        self.pulse_width = pulse_width

        pi.set_servo_pulsewidth(self.gpio_pin, self.pulse_width)
        
        ### This is to debug        
        #print("t {}\tpulse width :{}".format(cnt,self.pulse_width))

    def kill_esc(self):
        pi.set_servo_pulsewidth(self.gpio_pin, 0)



import time
import pigpio
pi = pigpio.pi()
#pi.set_servo_pulsewidth(ESC1, 0)

max_value = 2000
min_value = 1000

hover_pwm = 1400
