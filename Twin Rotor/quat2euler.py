import math
import numpy as np
def quat2euler_angle(w, x, y, z):
    ysqr = y*y
    
    t0 = 2.0 * (w*x + y*z)
    t1 = 1.0 - 2.0*(x*x + ysqr)
    ## Roll
    X = math.degrees(math.atan2(t0,t1))
    #X = math.atan2(t0,t1)

    t2 = 2.0 * (w*y - z*x)
    t2 = 1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    ## Pitch    
    Y = math.degrees(math.asin(t2))
    #Y = math.asin(t2)

    t3 = 2.0 * (w*z + x*y)
    t4 = 1.0 - 2.0 * (ysqr + z*z)
    ## Yaw    
    Z = math.degrees(math.atan2(t3,t4))
    #Z = math.atan2(t3,t4)
    
    return X,Y,Z


def wrap_angle(angle):
    ## supply angle in degrees
    if angle > 180:
        angle -= 2.*180
        
    return angle

def wrap_angle_radians(angle):
    ## supply angle in degrees
    if angle > np.pi:
        angle -= 2.*np.pi
        
    return angle
