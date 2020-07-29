##
import csv
import numpy as np
import matplotlib.pyplot as plt

data = []
with open('scripts/Caracterizacion rueda1 (izquierda).csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line0 = True
    for row in csv_reader:
        if line0:
            line0 = False
        else:
            data.append([float(row[0]), float(row[1]), float(row[2])])

data = np.array(data)
##

data.max()

##

plt.plot(data[:, 0], data[:, 2])
plt.plot(data[:, 0], data[:, 1])
plt.title('Respuesta escalon rueda 1')
plt.grid()
plt.ylim((-10, 1.05*data.max()))
plt.show()
##

