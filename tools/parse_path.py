import sys
import os
import string
import math
from math import pi

from matplotlib import pyplot as plt
import numpy as nm
from datetime import datetime
import array

script_dir=os.path.dirname(os.path.abspath(__file__));
filepath = sys.argv[1];

import xml.etree.ElementTree as ET
tree = ET.parse(filepath)
root = tree.getroot()

# all items data
x = [];
y = [];
h = [];
for elem in root:
    i = 0;
    for subelem in elem:
        if (i == 0):
            x.append(float(subelem.text))
        if (i == 1):
            y.append(float(subelem.text))
        if (i ==2):
            h.append(float(subelem.text))
        i = i + 1;

for i in range(len(x)):
    if (i == 6):
        x[i] = x[i - 1] + x[i];
        y[i] = y[i - 1] + y[i];
    if (i == 7):
        x[i] = x[i - 1] - x[i];
        y[i] = y[i - 1] - y[i];
    print("step[", i, "]", x[i], y[i], h[i])

im = plt.imread(script_dir+"\\skystone_field.png");
#plt.xlim([-100, 700])
#plt.ylim([-100, 700])
#plt.xticks([])
#plt.yticks([])
#plt.plot(data_x_raw, data_y_raw, label="actual path");
new_x = [];
new_y = [];
for i in range(len(x)):
    new_x.append(300 - x[i] * 100/24);
    new_y.append(300 - y[i] * 100/24);
    #print(new_x[i], new_y[i]);
    #plt.scatter(new_y[i], 600-new_x[i], zorder=2);
    #plt.plot(new_y[i], 600-new_x[i])
plt.plot(new_y, new_x)
plt.scatter(new_y, new_x, zorder=2);

implot = plt.imshow(im);

plt.show();
#plt.waitforbuttonpress(1); input();
#plt.close('all')
