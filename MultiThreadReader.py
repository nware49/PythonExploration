import threading
import queue
import logging
import time
import random
import serial
import csv
from datetime import datetime
import numpy as np

PortChannel1 = "/dev/ttySC0"
PortChannel2 = "/dev/ttySC1"

logging.basicConfig(level=logging.DEBUG, format='(%(threadName)-9s) %(message)s',)

try:
    Serial1 = serial.Serial(PortChannel1)
    Serial2 = serial.Serial(PortChannel2)
except:
    print("Could not open port. Port is either in use or does not exist.")
    exit()

BUF_SIZE = 10
q1 = queue.Queue(BUF_SIZE)
q2 = queue.Queue(BUF_SIZE)
breakIndicator = 0

class ProducerThreadChannel1(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ProducerThreadChannel1,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            curDT = datetime.now()
            TimeLong = curDT.strftime("%H:%M:%S:%f")
            Time = TimeLong[0:len(TimeLong)-3]
            if not q1.full():
                ser_bytes = Serial1.readline()
                DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                StringValues = (DecodedBytes + "," + Time)
                IncrementNum,TimeVal,RandFloat,SineVal,CosineVal,TimeStamp = StringValues.split(",")
                q1.put(TimeStamp + "," + SineVal)
                if breakIndicator == 1:
                    break
        return

class ProducerThreadChannel2(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ProducerThreadChannel2,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            curDT = datetime.now()
            TimeLong = curDT.strftime("%H:%M:%S:%f")
            Time = TimeLong[0:len(TimeLong)-3]
            if not q2.full():
                ser_bytes = Serial2.readline()
                DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                StringValues = (DecodedBytes + "," + Time)
                IncrementNum,TimeVal,RandFloat,SineVal,CosineVal,TimeStamp = StringValues.split(",")
                q2.put(TimeStamp + "," + CosineVal)
                if breakIndicator == 1:
                    break
        return

try:
    if __name__ == '__main__':
        
        pc1 = ProducerThreadChannel1(name='producerChannel1')
        pc2 = ProducerThreadChannel2(name='producerChannel2')

        pc1.start()
        pc2.start()

except:
    print("An error occured, the threads could not be spawned.")
    exit()

IntDataArr = [[0,0,0],[0,0,0]]

while True:
    try:
        if q1.empty() and q2.empty():
            time.sleep(0.1)
            continue
        if not q1.empty():
            Data1 = q1.get()
            TimeStamp1,Sine = Data1.split(",")
            with open("SineCosine.csv","a") as file:
                #Opens this file^ Writes data here. 
                writer = csv.writer(file, delimiter=",")
                writer.writerow(("Channel 1 Data",TimeStamp1,Sine))
        if not q2.empty():
            Data2 = q2.get()
            TimeStamp2,Cosine = Data2.split(",")
            with open("SineCosine.csv","a") as file:
                #Opens this file^ Writes data here. 
                writer = csv.writer(file, delimiter=",")
                writer.writerow(("Channel 2 Data",TimeStamp2,Cosine))
                
        IntDataArr[0][1] = TimeStamp2
        IntDataArr[0][2] = TimeStamp1
        IntDataArr[1][1] = Cosine
        Interpolation = (float(IntDataArr[1][0]) + ((float(Cosine) - float(IntDataArr[1][0])) / (float(TimeStamp2) - float(IntDataArr[0][0]))) *  (float(TimeStamp1) - float(IntDataArr[0][0])))
        IntDataArr[1][2] = Interpolation

        for r in IntDataArr:
            for c in r:
                print(c,end = " ")
            print()

        IntDataArr[0][0] = TimeStamp2
        IntDataArr[1][0] = Cosine
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        breakIndicator = 1
        exit()
    except:
        print("An error occurred, data could not be recieved.")
        exit()
