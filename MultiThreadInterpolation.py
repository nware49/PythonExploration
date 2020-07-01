import threading
import collections
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
            Time = time.time()
            if not q1.full():
                ser_bytes = Serial1.readline()
                DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                StringValues = (DecodedBytes + "," + str(Time))
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
            Time = time.time()
            if not q2.full():
                ser_bytes = Serial2.readline()
                DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                StringValues = (DecodedBytes + "," + str(Time))
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

IntDataArrTime = collections.deque()
IntDataArrCosine = collections.deque()
IntDataArrSineTime = collections.deque()

while True:
    try:
        if q1.empty() or q2.empty():
            time.sleep(0.1)
            continue
        
        if not q1.empty() and not q2.empty():
            Data1 = q1.get()
            TimeStampSine,Sine = Data1.split(",")
            SineTime = time.ctime(float(TimeStampSine))
            Data2 = q2.get()
            TimeStampCosine,Cosine = Data2.split(",")
            CosineTime = time.ctime(float(TimeStampCosine))
            
        if not q1.empty() and q2.empty():
            Data1 = q1.get()
            TimeStampSine,Sine = Data1.split(",")
            SineTime = time.ctime(float(TimeStampSine))
            
        if not q2.empty() and q1.empty():
            Data2 = q2.get()
            TimeStampCosine,Cosine = Data2.split(",")
            CosineTime = time.ctime(float(TimeStampCosine))
            
        IntDataArrTime.append(float(TimeStampCosine))
        IntDataArrCosine.append(float(Cosine))
        IntDataArrSineTime.append(float(TimeStampSine))

        ValidInterp = True
        
        if len(IntDataArrTime)>5:
            IntDataArrTime.popleft()
        if len(IntDataArrCosine)>5:
            IntDataArrCosine.popleft()
        if len(IntDataArrSineTime)>5:
            IntDataArrSineTime.popleft()
        try:
            Interpolation = (IntDataArrCosine[-3]) + (((IntDataArrCosine[-2]) - (IntDataArrCosine[-3])) / ((IntDataArrTime[-2]) - (IntDataArrTime[-3]))) *  ((IntDataArrSineTime[-2]) - (IntDataArrTime[-3]))
        except:
            print("Not enough data to interpolate. Continuing.")
            continue
        with open("Interpolation.csv","a") as file:
            #Opens this file^ Writes data here. 
            writer = csv.writer(file, delimiter=",")
            writer.writerow(("Channel 1 Data",SineTime,Sine,"|","Channel 2 Data",CosineTime,Cosine,"Interp:" + Interpolation,ValidInterp))
        print(Interpolation)
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        breakIndicator = 1
        exit()
    except:
        print("An error occurred, data could not be recieved.")
        exit()
