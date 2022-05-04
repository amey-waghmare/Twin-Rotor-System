import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


data = pd.read_csv("pwm_speed__1.csv")
data = np.array(data)

Y = data[:,-1]

x1 = data[:,0]
x2 = np.power(data[:,0],2)

X = np.transpose(np.vstack([x1,x2]))

print(X)