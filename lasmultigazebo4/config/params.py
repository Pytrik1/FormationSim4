#!/usr/bin/env python
import numpy as np

#Controller gains
c = np.float32(0.5)
cf = np.float(1.2)
cD =np.float(2.0)
cI = np.float(0.6)
cP = np.float(4)
calpha = np.float(40)
czeta = np.float(4)

#Goal position of centroid
pGoal = np.array([[-2.0],[-2.0]])
r_safe = np.float32(0.8)

v_min = 0.1
v_max = 0.5
Ug_lim = 0.25
h = 0.005

cO = 5