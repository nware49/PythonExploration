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
except:
    print("Not connected to PortChannel2. Port is either in use or does not exist.")
    #exit()

try:
    Serial2 = serial.Serial(PortChannel2)
except:
    print("Not connected to PortChannel2. Port is either in use or does not exist.")
    #exit()

BUF_SIZE = 10
q1 = queue.Queue(BUF_SIZE)
q2 = queue.Queue(BUF_SIZE)
breakIndicator = False

class ProducerThreadChannel1(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(ProducerThreadChannel1,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            Time = time.time()
            if not q1.full():
                try:
                    ser_bytes = Serial1.readline()
                    DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                    StringValues = (DecodedBytes + "," + str(Time))
                    IncrementNum,TimeVal,RandFloat,SineVal,CosineVal,TimeStamp = StringValues.split(",")
                    q1.put(TimeStamp + "," + SineVal)
                    if breakIndicator == True:
                        break
                except:
                    print("Could not retrieve data from Port1.")
                    time.sleep(0.1)
                    continue
        return

class ProducerThreadChannel2(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(ProducerThreadChannel2,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            Time = time.time()
            if not q2.full():
                try:
                    ser_bytes = Serial2.readline()
                    DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                    StringValues = (DecodedBytes + "," + str(Time))
                    IncrementNum,TimeVal,RandFloat,SineVal,CosineVal,TimeStamp = StringValues.split(",")
                    q2.put(TimeStamp + "," + CosineVal)
                    if breakIndicator == True:
                        break
                except:
                    print("Could not retrieve data from Port2.")
                    time.sleep(0.1)
                    continue
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

IntDataArrFormSineTime = collections.deque()
IntDataArrSineTime = collections.deque()
IntDataArrSine = collections.deque()

IntDataArrFormCosineTime = collections.deque()
IntDataArrCosineTime = collections.deque()
IntDataArrCosine = collections.deque()

while True:
    try:
        if q1.empty() and q2.empty():
            time.sleep(0.1)
            continue
        
        if q1.empty() or q2.empty():
            time.sleep(0.55)
            
        if not q1.empty() and not q2.empty():
            print(3)
            Data1 = q1.get()
            TimeStampSine,Sine = Data1.split(",")
            SineTime = time.ctime(float(TimeStampSine))
            Data2 = q2.get()
            TimeStampCosine,Cosine = Data2.split(",")
            CosineTime = time.ctime(float(TimeStampCosine))

            IntDataArrSineTime.append(float(TimeStampSine))
            IntDataArrSine.append(float(Sine))
            IntDataArrFormSineTime.append(str(SineTime))
            if len(IntDataArrSineTime)>5:
                IntDataArrSineTime.popleft()
            if len(IntDataArrSine)>5:
                IntDataArrSine.popleft()
            if len(IntDataArrFormSineTime)>5:
                IntDataArrFormSineTime.popleft()
            
            IntDataArrCosineTime.append(float(TimeStampCosine))
            IntDataArrCosine.append(float(Cosine))
            IntDataArrFormCosineTime.append(str(CosineTime))
            if len(IntDataArrCosineTime)>5:
                IntDataArrCosineTime.popleft()
            if len(IntDataArrCosine)>5:
                IntDataArrCosine.popleft()
            if len(IntDataArrFormCosineTime)>5:
                IntDataArrFormCosineTime.popleft()

            ValidInterp = True
            
            try:
                LastFormSineTime = IntDataArrFormSineTime[-2]
                LastSine = IntDataArrSine[-2]
                LastFormCosineTime = IntDataArrFormCosineTime[-2]
                LastCosine = IntDataArrCosine[-2]
                Interpolation = (IntDataArrCosine[-3]) + (((IntDataArrCosine[-2]) - (IntDataArrCosine[-3])) / ((IntDataArrCosineTime[-2]) - (IntDataArrCosineTime[-3]))) *  ((IntDataArrSineTime[-2]) - (IntDataArrCosineTime[-3]))
                ShortInterp = format(Interpolation, '.4f')
                with open("Interpolation.csv","a") as file:
                    #Opens this file^ Writes data here. 
                    writer = csv.writer(file, delimiter=",")
                    writer.writerow(("Channel 1 Data",LastFormSineTime,LastSine,"Channel 2 Data",LastFormCosineTime,LastCosine,"Interp:" + str(ShortInterp),ValidInterp))
                print(Interpolation)
            except:                
                ValidInterp = False
                print("Not enough data to interpolate. Continuing.")
                with open("Interpolation.csv","a") as file:
                    #Opens this file^ Writes data here. 
                    writer = csv.writer(file, delimiter=",")
                    writer.writerow(("Not","enough","data.","x","x","x","x",ValidInterp))
            continue
            
            
        if not q1.empty() and q2.empty():
            print(1)
            ValidInterp = False
            Data1 = q1.get()
            TimeStampSine,Sine = Data1.split(",")
            SineTime = time.ctime(float(TimeStampSine))
            IntDataArrSineTime.append(float(TimeStampSine))
            IntDataArrSine.append(float(Sine))
            IntDataArrFormSineTime.append(str(SineTime))
            if len(IntDataArrSineTime)>5:
                IntDataArrSineTime.popleft()
            if len(IntDataArrSine)>5:
                IntDataArrSine.popleft()
            if len(IntDataArrFormSineTime)>5:
                IntDataArrFormSineTime.popleft()
            try:
                LastFormSineTime = IntDataArrSineTime[-2]
                LastSine = IntDataArrSine[-2]
            except:
                print("Not enough data to interpolate. Continuing.")
                continue
            with open("Interpolation.csv","a") as file:
                #Opens this file^ Writes data here. 
                writer = csv.writer(file, delimiter=",")
                writer.writerow(("Channel 1 Data",LastFormSineTime,LastSine,"x","x","x","x",ValidInterp))
            continue
        
        if not q2.empty() and q1.empty():
            print(2)
            ValidInterp = False
            Data2 = q2.get()
            TimeStampCosine,Cosine = Data2.split(",")
            CosineTime = time.ctime(float(TimeStampCosine))
            IntDataArrCosineTime.append(float(TimeStampCosine))
            IntDataArrCosine.append(float(Cosine))
            IntDataArrFormCosineTime.append(str(CosineTime))
            if len(IntDataArrCosineTime)>5:
                IntDataArrCosineTime.popleft()
            if len(IntDataArrCosine)>5:
                IntDataArrCosine.popleft()
            if len(IntDataArrFormCosineTime)>5:
                IntDataArrFormCosineTime.popleft()
            try:
                LastFormCosineTime = IntDataArrCosineTime[-2]
                LastCosine = IntDataArrCosine[-2]
            except:
                print("Not enough data to interpolate. Continuing.")
                continue
            with open("Interpolation.csv","a") as file:
                #Opens this file^ Writes data here. 
                writer = csv.writer(file, delimiter=",")
                writer.writerow(("x","x","x","Channel 2 Data",LastFormCosineTime,LastCosine,"x",ValidInterp))
            continue
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        breakIndicator = True
        exit()
    except:
        print("An error occurred, data could not be recieved.")
        exit()
