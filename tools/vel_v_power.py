import sys
import os
import string
import math
from datetime import datetime
from matplotlib import pyplot as plt
import numpy as np

def get_time(t):
    t = t.split(' ')
    #print(t)
    t_s = ' '
    t_s = t_s.join(t[:2])
    #print(t_s)
    t = datetime.strptime(t_s, '%m-%d  %H:%M:%S.%f')
    return t

def get_sec(time_str):
    """Get Seconds from time."""
    m, s = time_str.split(':')
    return int(m) * 60 + (float(s))

script_dir=os.path.dirname(os.path.abspath(__file__))

data_velocity = [[], [], [], []] #bl, br, fl, fr
data_power = [[], [], [], []]
power_time = [[], [], [], []]
velocity_time = [[],[],[],[]]
data_avg_velocity = [[], [], [], []]
data_avg_power = [[], [], [], []]
count = 0
initial_time = 0
file = open(sys.argv[1])

for line in file:
    if(' D ' in line) and not("Terminate batch job" in line):
        if("Power" in line):
            if("backLeftPower" in line):
                if count == 0:
                    initial_time = get_time(line.split("4250")[0].strip())
                    count  = 1
                data_power[0].append(float(line.split(" ")[-1].strip()))
                power_time[0].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))
            elif("backRightPower" in line):
                data_power[1].append(float(line.split(" ")[-1].strip()))
                power_time[1].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))
            elif("frontLeftPower" in line):
                data_power[2].append(float(line.split(" ")[-1].strip()))
                power_time[2].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))
            elif("frontRightPower" in line):
                data_power[3].append(float(line.split(" ")[-1].strip()))
                power_time[3].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))

            
        ##########################################
        if("Velocity" in line):
            if("backLeftVelocity" in line):
                data_velocity[0].append(float(line.split(" ")[-1].strip()))
                velocity_time[0].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))
            elif("backRightVelocity" in line):
                data_velocity[1].append(float(line.split(" ")[-1].strip()))
                velocity_time[1].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))
            elif("frontLeftVelocity" in line):
                data_velocity[2].append(float(line.split(" ")[-1].strip()))
                velocity_time[2].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))
            elif("frontRightVelocity" in line):
                data_velocity[3].append(float(line.split(" ")[-1].strip()))
                velocity_time[3].append(float(get_sec(str((get_time(line.split("4250")[0].strip())-initial_time))[3:])))

file.close()

print(data_power[3])
plt.figure()
plt.title("power vs time")
for i in range(4):
    plt.plot(np.array(power_time[i]), np.array(data_power[i]))
plt.legend(["backLeft", "backRight", "frontLeft", "frontRight"], loc='upper left')
plt.ylabel('power')
plt.xlabel('time')

plt.figure()
plt.title("velocity vs time")
for i in range(4):
    plt.plot(np.array(velocity_time[i]), np.array(data_velocity[i]))
plt.legend(["backLeft", "backRight", "frontLeft", "frontRight"], loc='upper left')
plt.ylabel('velocity')
plt.xlabel('time')
for i in range(4):
    if len(data_velocity[i]) > len(data_power[i]):
        for j in range(len(data_velocity[i]) - len(data_power[i])):
            data_velocity[i].pop(-1)
    elif len(data_velocity[i]) < len(data_power[i]):
        for j in range(len(data_power[i]) - len(data_velocity[i])):
            data_power[i].pop(-1)
"""
for i in range(len(data_velocity)):
    vavg = 0
    pavg = 0
    for j in range(3):
        vavg += data_velocity[j][i]
        pavg += data_power[j][i]

    
    vavg /= 4
    pavg /= 4

    data_avg_velocity.append(vavg)
    data_avg_power.append(pavg)
"""
plt.figure()
plt.title("four motors")
for i in range(4):
    plt.plot(np.array(data_velocity[i]), np.array(data_power[i]))
plt.legend(["backLeft", "backRight", "frontLeft", "frontRight"], loc='upper left')
plt.ylabel('power')
plt.xlabel('velocity')
"""
plt.figure()
plt.title("average")
plt.plot(data_avg_velocity, data_avg_power)
plt.ylabel('power')
plt.xlabel('velocity')
"""
plt.show()