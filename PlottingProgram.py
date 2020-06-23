import csv
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

x = []
y = []

with open('csvfile2.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    #data = list(plots)
    #rowCount = len(data)
    #print(rowCount)
    #print(data)
    for row in plots:
        x.append(int(row[0]))
        y.append(float(row[1]))

print(x)
print(y)

plt.plot(x,y, marker= "o")

plt.title("Data from Simulator")
plt.xlabel("Time")
plt.ylabel("SineWave")
          
plt.show()
