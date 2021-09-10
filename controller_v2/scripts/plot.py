import sys
import random

import matplotlib.pyplot as plt
from matplotlib import style

log_file = sys.argv[1]

with open(log_file) as f:
    lines = f.readlines()

time_list = []
p_list = []
r_list = []
y_list = []
for n, line in enumerate(lines):
    p, r, y = line.split(",")
    time_list.append(n)
    p_list.append(int(p)/100)
    r_list.append(int(r)/100)
    y_list.append(int(y)/100)




fig = plt.figure()
a_p = fig.add_subplot(3, 1, 1)
a_r = fig.add_subplot(3, 1, 2)
a_y = fig.add_subplot(3, 1, 3)

a_p.set_ylim(-45, 45)
a_r.set_ylim(-45, 45)
a_y.set_ylim(-45, 45)

a_p.plot(time_list, p_list)
a_r.plot(time_list, r_list)
a_y.plot(time_list, y_list)

plt.show()
