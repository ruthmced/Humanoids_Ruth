#!/usr/bin/env python
import json
from matplotlib import pyplot as plt   

with open('kalman_data_y.txt') as positions_data:
    all_positions = positions_data.readlines()
    data_number = len(all_positions)

time  = 0
index = 0

time_axis  = []
exact_position_y = []
measurent_position_y = []
prediction_position_y = []
estimation_position_y = []

print "Number of positions loaded->", data_number 

for position in all_positions:
    axis = json.loads(all_positions[index])

    exact_position_y.append(axis[0])
    measurent_position_y.append(axis[1])
    prediction_position_y.append(axis[2])
    estimation_position_y.append(axis[3])

    index += 1
    time  += 0.03
    time_axis.append(time)   


plt.figure(2)
plt.plot(time_axis, exact_position_y, 'k.-', time_axis, measurent_position_y, 'b.', time_axis, prediction_position_y, 'g*', time_axis, estimation_position_y, 'r--+')
#plt.plot(time_axis, exact_position_y, 'k.-', time_axis, measurent_position_y, 'b.', time_axis, estimation_position_y, 'r+')
#plt.plot(time_axis, exact_position_y, 'k.-', time_axis, measurent_position_y, 'b.')
#plt.plot(time_axis, exact_position_y, 'k.-', time_axis, estimation_position_y, 'r+')

plt.xlabel('Tiempo [s]')
plt.ylabel('Posicion [m]')
plt.title('Tiempo vs Posicion')
plt.legend(('Real', 'Medicion', 'Prediccion', 'Correccion'), loc = 'upper left')
#plt.legend(('Real', 'Medicion', 'Estimacion'), loc = 'upper left')
#plt.legend(('real', 'measured'), loc = 'upper left')
#plt.legend(('real', 'corrected'), loc = 'upper left')

plt.show()