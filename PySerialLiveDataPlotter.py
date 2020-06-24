import serial
import time
import csv
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.animation import FuncAnimation
Port2 = "COM4"  #This is which port the data will be recieved at, can be changed
#Attempts to open and assign a serial port
#If it cannot open the port, it will print an error message
try:
    ser2 = serial.Serial(Port2)
except:
    print("Could not open port. Port is either in use or does not exist.")
    exit()
print(ser2.name)

xVar = []
yVar = []

IncrementNum = 0
SineVal = 0

def animate(i):
    try:
        ser_bytes = ser2.readline()
        DecodedBytes = ser_bytes[0:(len(ser_bytes)-1)].decode("utf-8")
        print(DecodedBytes)
        IncrementNum,TimeVal,RandFloat,SineVal,CosineVal = DecodedBytes.split(",")
        xVar.append(int(IncrementNum))
        yVar.append(float(SineVal))
        plt.cla()
        plt.plot(xVar, yVar, marker= "o")
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        exit()
    except:
        print("An error occurred, data could not be recieved.")
        exit()
plt.title("Sine Wave Simulation")
plt.xlabel("Time")
plt.ylabel("SineVal")
animation = FuncAnimation(plt.gcf(), animate)
plt.tight_layout()
plt.show()
