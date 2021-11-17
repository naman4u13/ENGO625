from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import json 
from collections import OrderedDict

f = open('/content/satMap.json')
data = json.load(f)
data = OrderedDict(sorted(data.items()))
t0 = int(list(data.keys())[0])
satPos = {}
for t,sats in data.items():
  # Remove the following 'if' block to plot for whole dataset
  if(int(t)-t0>300):
    break
  for sat in sats:
    ecef = sat["ecef"]
    prn = sat["prn"]
    satPos.setdefault(prn, []).append(ecef)
    

remote = [-1633489.3796772808, -3651627.1825036877, 4952481.619548985]
base = [-1625352.1365678546, -3653483.7341981716, 4953733.892846955]

fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(1,1,1, projection='3d')
ax.scatter(remote[0],remote[1],remote[2],label="Remote Station",marker='x',s=100)
for i, (k, v) in enumerate(satPos.items()):
      
  # code to split it into 3 lists
  x,y,z = map(list, zip(*v))
  ax.scatter(x,y,z,label=k,marker='o')
  
ax.set_title(str(k))
ax.set_xlabel('x')                         
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.legend(loc='upper left')
plt.show()
  