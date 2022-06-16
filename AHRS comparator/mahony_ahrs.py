import numpy as np


### Some HElper Functions

def am2DCM(a: np.ndarray, m: np.ndarray, frame: str = 'ENU') -> np.ndarray:
    if frame.upper() not in ['ENU', 'NED']:
        raise ValueError("Wrong coordinate frame. Try 'ENU' or 'NED'")
    a = np.array(a)
    m = np.array(m)
    H = np.cross(m, a)
    H /= np.linalg.norm(H)
    a /= np.linalg.norm(a)
    M = np.cross(a, H)
    if frame.upper()=='ENU':
        return np.array([[H[0], M[0], a[0]],
                         [H[1], M[1], a[1]],
                         [H[2], M[2], a[2]]])
    return np.array([[M[0], H[0], -a[0]],
                     [M[1], H[1], -a[1]],
                     [M[2], H[2], -a[2]]])

def dcm2quat(R: np.ndarray) -> np.ndarray:
    if(R.shape[0] != R.shape[1]):
        raise ValueError('Input is not a square matrix')
    if(R.shape[0] != 3):
        raise ValueError('Input needs to be a 3x3 array or matrix')
    q = np.array([1., 0., 0., 0.])
    q[0] = 0.5*np.sqrt(1.0 + R.trace())
    q[1] = (R[1, 2] - R[2, 1]) / q[0]
    q[2] = (R[2, 0] - R[0, 2]) / q[0]
    q[3] = (R[0, 1] - R[1, 0]) / q[0]
    q[1:] /= 4.0
    return q / np.linalg.norm(q)



def q_prod(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    pq = np.zeros(4)
    pq[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3]
    pq[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2]
    pq[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1]
    pq[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]
    
    return pq

def acc2q(a: np.ndarray, return_euler: bool = False) -> np.ndarray:
    ## Return Quaternion from Accelerometer
    q = np.array([1.0, 0.0, 0.0, 0.0])
    ex, ey, ez = 0.0, 0.0, 0.0
    if np.linalg.norm(a)>0 and len(a)==3:
        ax, ay, az = a
        # Normalize accelerometer measurements
        a_norm = np.linalg.norm(a)
        ax /= a_norm
        ay /= a_norm
        az /= a_norm
        # Euler Angles from Gravity vector
        ex = np.arctan2(ay, az)
        ey = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        ez = 0.0
        if return_euler:
            return np.array([ex, ey, ez])*RAD2DEG
        # Euler to Quaternion
        cx2 = np.cos(ex/2.0)
        sx2 = np.sin(ex/2.0)
        cy2 = np.cos(ey/2.0)
        sy2 = np.sin(ey/2.0)
        q = np.array([cx2*cy2, sx2*cy2, cx2*sy2, -sx2*sy2])
        q /= np.linalg.norm(q)
    return q


def am2q(a: np.ndarray, m: np.ndarray, frame: str = 'ENU') -> np.ndarray:
    ## Return Quaternion from Accelerometer and Magnetometer by TRAID method
    R = am2DCM(a, m, frame=frame)
    q = dcm2quat(R)
    return q

def q2R(q: np.ndarray) -> np.ndarray:
    if q is None:
        return np.identity(3)
    if q.shape[-1]!= 4:
        raise ValueError("Quaternion Array must be of the form (4,) or (N, 4)")
    if q.ndim>1:
        q /= np.linalg.norm(q, axis=1)[:, None]     # Normalize all quaternions
        R = np.zeros((q.shape[0], 3, 3))
        R[:, 0, 0] = 1.0 - 2.0*(q[:, 2]**2 + q[:, 3]**2)
        R[:, 1, 0] = 2.0*(q[:, 1]*q[:, 2]+q[:, 0]*q[:, 3])
        R[:, 2, 0] = 2.0*(q[:, 1]*q[:, 3]-q[:, 0]*q[:, 2])
        R[:, 0, 1] = 2.0*(q[:, 1]*q[:, 2]-q[:, 0]*q[:, 3])
        R[:, 1, 1] = 1.0 - 2.0*(q[:, 1]**2 + q[:, 3]**2)
        R[:, 2, 1] = 2.0*(q[:, 0]*q[:, 1]+q[:, 2]*q[:, 3])
        R[:, 0, 2] = 2.0*(q[:, 1]*q[:, 3]+q[:, 0]*q[:, 2])
        R[:, 1, 2] = 2.0*(q[:, 2]*q[:, 3]-q[:, 0]*q[:, 1])
        R[:, 2, 2] = 1.0 - 2.0*(q[:, 1]**2 + q[:, 2]**2)
        return R
    q /= np.linalg.norm(q)
    return np.array([
        [1.0-2.0*(q[2]**2+q[3]**2), 2.0*(q[1]*q[2]-q[0]*q[3]), 2.0*(q[1]*q[3]+q[0]*q[2])],
        [2.0*(q[1]*q[2]+q[0]*q[3]), 1.0-2.0*(q[1]**2+q[3]**2), 2.0*(q[2]*q[3]-q[0]*q[1])],
        [2.0*(q[1]*q[3]-q[0]*q[2]), 2.0*(q[0]*q[1]+q[2]*q[3]), 1.0-2.0*(q[1]**2+q[2]**2)]])




class Mahony:
    def __init__(self,
        gyr: np.ndarray = None,
        acc: np.ndarray = None,
        mag: np.ndarray = None,
        frequency: float = 100.0,
        k_P: float = 1.0,
        k_I: float = 0.3,
        q0: np.ndarray = None,
        **kwargs):
        self.gyr = gyr
        self.acc = acc
        self.mag = mag
        self.frequency = frequency
        self.q0 = q0
        self.k_P = k_P
        self.k_I = k_I
        # Old parameter names for backward compatibility
        self.k_P = kwargs.get('kp', k_P)
        self.k_I = kwargs.get('ki', k_I)
        self.Dt = kwargs.get('Dt', 1.0/self.frequency)
        
        self.eInt = [0.0,0.0,0.0]
        




        # Estimate all orientations if sensor data is given
        if self.gyr is not None and self.acc is not None:
            self.Q = self._compute_all()

    def _compute_all(self):
        if self.acc.shape != self.gyr.shape:
            raise ValueError("acc and gyr are not the same size")
        num_samples = len(self.gyr)
        Q = np.zeros((num_samples, 4))
        
        # Compute with MARG Architecture
        if self.mag.shape != self.gyr.shape:
            raise ValueError("mag and gyr are not the same size")
        Q[0] = am2q(self.acc[0], self.mag[0]) if self.q0 is None else self.q0/np.linalg.norm(self.q0)
        for t in range(1, num_samples):
            Q[t] = self.updateMARG(Q[t-1], self.gyr[t], self.acc[t], self.mag[t])
        return Q


    def updateMARG(self, q: np.ndarray, gyr: np.ndarray, acc: np.ndarray, mag: np.ndarray) -> np.ndarray:
        if gyr is None or not np.linalg.norm(gyr)>0:
            return q
        Omega = np.copy(gyr)
        a_norm = np.linalg.norm(acc)
        if a_norm>0:
            m_norm = np.linalg.norm(mag)
            if not m_norm>0:
                return self.updateIMU(q, gyr, acc)
            a = np.copy(acc)/a_norm
            m = np.copy(mag)/m_norm
            R = q2R(q)
            v_a = R.T@np.array([0.0, 0.0, 1.0])     # Expected Earth's gravity
            # Rotate magnetic field to inertial frame
            h = R@m
            v_m = R.T@np.array([-np.linalg.norm([h[0], h[1]]), 0.0, h[2]])
            v_m /= np.linalg.norm(v_m)
            
            omega_mes = np.cross(a, v_a) + np.cross(m, v_m) # Cost function (eqs. 32c and 48a)
            self.eInt = self.eInt + omega_mes * self.Dt             # Estimated Gyro bias (eq. 48c)
            
            Omega = Omega + self.k_P*omega_mes + self.k_I * self.eInt   # Gyro correction
        p = np.array([0.0, *Omega])
        qDot = 0.5*q_prod(q, p)                     # Rate of change of quaternion (eqs. 45 and 48b)
        q += qDot*self.Dt                           # Update orientation
        q /= np.linalg.norm(q)                      # Normalize Quaternion (Versor)
        return q












