import threading
import collections
import queue
import logging
import time
import random
import serial
import csv
import re
from datetime import datetime
import numpy as np

PortChannel1 = "/dev/ttySC0"
PortChannel2 = "/dev/ttySC1"

#Formatting CSV Files
##with open("XMissDataFile.csv","a") as file: 
##    writer = csv.writer(file, delimiter=",")
##    writer.writerow(("XMiss Sample Time","Absorbance"))
##with open("CTDDataFile.csv","a") as file: 
##    writer = csv.writer(file, delimiter=",")
##    writer.writerow(("CTD Sample Time","Temperature"))
##with open("CSDF.csv","a") as file: 
##    writer = csv.writer(file, delimiter=",")
##    writer.writerow(("XMiss Sample Time","Absorbance","CTD Sample Time","Temperature","Interpolation","ValidInterp"))
##with open("Interpolation.csv","a") as file: 
##    writer = csv.writer(file, delimiter=",")
##    writer.writerow(("XMiss Sample Time","CTD Sample Time","Interpolation"))

logging.basicConfig(level=logging.DEBUG, format='(%(threadName)-9s) %(message)s',)

try:
    Serial1 = serial.Serial(PortChannel1)
    Serial1.baudrate = 19200
except:
    print("Not connected to PortChannel1. Port is either in use or does not exist.")
    #exit()

try:
    Serial2 = serial.Serial(PortChannel2)
    Serial2.baudrate = 9600

    Serial2.write(b'\r')
    CTDReadline = Serial2.readline()

    if CTDReadline == b'S>':
        Serial2.write(b'startnow\r')
        print("Started CTD.")
    if CTDReadline == b'SBE 39\r\n':
        Serial2.write(b'startnow\r')
        print("Started CTD.")
    else:
        print("CTD is already running.")
    
except Exception as Exc:
    print(Exc)
    print("Not connected to PortChannel2. Port is either in use or does not exist.")
    exit()

BUF_SIZE = 10
qXMiss = queue.Queue(BUF_SIZE)
qCTD = queue.Queue(BUF_SIZE)
breakIndicator = False

class ProducerThreadChannel1(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(ProducerThreadChannel1,self).__init__()
        self.target = target
        self.name = name
        Serial1.flush()

    def run(self):
        Serial1.flush()
        while True:
            Time = time.time()
            if not qXMiss.full():
                try:
                    ser_bytes = Serial1.readline()
                    DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                    CompressedBytes = re.sub("\s+"," ",DecodedBytes)
                    StringValues = (CompressedBytes + " " + str(Time))
                    Name,XMiss1,XMiss2,XMiss3,Absorbance,Wavelength,TimeStamp = StringValues.split(" ")
                    qXMiss.put(TimeStamp + "," + Absorbance)
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
        Serial2.flush()

    def run(self):
        Serial2.flush()
        while True:
            Time = time.time()
            if not qCTD.full():
                try:
                    ser_bytes = Serial2.readline()
                    if ser_bytes == b'\r\n':
                        continue
                    DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                    if DecodedBytes == "S>":
                        continue
                    if DecodedBytes == "S>startnow":
                        continue
                    CompressedBytes = re.sub("\s+"," ",DecodedBytes)
                    StringValues = (CompressedBytes + ", " + str(Time))
                    HashTemperature,Depth,CTDDate,CTDTime,TimeStamp = StringValues.split(", ")
                    Temperature = HashTemperature[-len(HashTemperature)+2:-1]
                    qCTD.put(TimeStamp + "," + Temperature)
                    if breakIndicator == True:
                        break
                except Exception as Exc:
                    print(Exc)
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

except Exception as Exc:
    print(Exc)
    print("An error occured, the threads could not be spawned.")
    exit()

IntDataArrFormXMissTime = collections.deque()
IntDataArrXMissTime = collections.deque()
IntDataArrXMiss = collections.deque()

IntDataArrFormCTDTime = collections.deque()
IntDataArrCTDTime = collections.deque()
IntDataArrCTD = collections.deque()

#Consumer While Loop
while True:
    if breakIndicator == True:
        break
    if len(IntDataArrXMissTime)>5:
        IntDataArrXMissTime.popleft()
    if len(IntDataArrXMiss)>5:
        IntDataArrXMiss.popleft()
    if len(IntDataArrFormXMissTime)>5:
        IntDataArrFormXMissTime.popleft()
    if len(IntDataArrCTDTime)>5:
        IntDataArrCTDTime.popleft()
    if len(IntDataArrCTD)>5:
        IntDataArrCTD.popleft()
    if len(IntDataArrFormCTDTime)>5:
        IntDataArrFormCTDTime.popleft()
    try:
        if qXMiss.empty() and qCTD.empty():
            time.sleep(0.125)
            continue
        
        if not qCTD.empty() and qXMiss.empty():
            print("XMiss Empty. May happen on startup.")
            ValidInterp = False
            DataCTD = qCTD.get()
            TimeStampCTD,Temperature = DataCTD.split(",")
            FormCTDTime = time.ctime(float(TimeStampCTD))
            IntDataArrCTDTime.append(float(TimeStampCTD))
            IntDataArrCTD.append(float(Temperature))
            IntDataArrFormCTDTime.append(str(FormCTDTime))
            with open("CSDF.csv","a") as file:
                #Opens this file^ Writes data here. 
                writer = csv.writer(file, delimiter=",")
                writer.writerow(("x","x",FormCTDTime,Temperature,"x",ValidInterp))
            with open("CTDDataFile.csv","a") as file:
                #Opens this file^ Writes data here. 
                writer = csv.writer(file, delimiter=",")
                writer.writerow((FormCTDTime,Temperature))
            continue

        if not qXMiss.empty() and qCTD.empty():
            try:
                DataCTD = qCTD.get(timeout=2)
            except KeyboardInterrupt:
                print("Keyboard Interrupt")
                breakIndicator = True
                break
            except:
                print("CTD Timeout")
                while not qXMiss.empty():
                    DataXMiss = qXMiss.get()
                    TimeStampXMiss,Absorbance = DataXMiss.split(",")
                    FormXMissTime = time.ctime(float(TimeStampXMiss))
                    IntDataArrXMissTime.append(float(TimeStampXMiss))
                    IntDataArrXMiss.append(float(Absorbance))
                    IntDataArrFormXMissTime.append(str(FormXMissTime))

                    with open("XMissDataFile.csv","a") as file:
                        #Opens this file^ Writes data here. 
                        writer = csv.writer(file, delimiter=",")
                        writer.writerow((FormXMissTime,Absorbance))
                continue
        
            else:
                ValidInterp = True
                TimeStampCTD,Temperature = DataCTD.split(",")
                FormCTDTime = time.ctime(float(TimeStampCTD))
                IntDataArrCTDTime.append(float(TimeStampCTD))
                IntDataArrCTD.append(float(Temperature))
                IntDataArrFormCTDTime.append(str(FormCTDTime))
                while not qXMiss.empty():
                    DataXMiss = qXMiss.get()
                    TimeStampXMiss,Absorbance = DataXMiss.split(",")
                    FormXMissTime = time.ctime(float(TimeStampXMiss))
                    IntDataArrXMissTime.append(float(TimeStampXMiss))
                    IntDataArrXMiss.append(float(Absorbance))
                    IntDataArrFormXMissTime.append(str(FormXMissTime))

                    with open("XMissDataFile.csv","a") as file:
                        #Opens this file^ Writes data here. 
                        writer = csv.writer(file, delimiter=",")
                        writer.writerow((FormXMissTime,Absorbance))
                    with open("CTDDataFile.csv","a") as file:
                        #Opens this file^ Writes data here. 
                        writer = csv.writer(file, delimiter=",")
                        writer.writerow((FormCTDTime,Temperature))
                        
                    try:
                        Interpolation = (IntDataArrCTD[-2]) + (((IntDataArrCTD[-1]) - (IntDataArrCTD[-2])) / ((IntDataArrCTDTime[-1]) - (IntDataArrCTDTime[-2]))) *  ((IntDataArrXMissTime[-1]) - (IntDataArrCTDTime[-2]))
                        ShortInterp = format(Interpolation, '.3f')
                        with open("CSDF.csv","a") as file:
                            #Opens this file^ Writes data here. 
                            writer = csv.writer(file, delimiter=",")
                            writer.writerow((FormXMissTime,Absorbance,FormCTDTime,Temperature,ShortInterp,ValidInterp))

                        with open("Interpolation.csv","a") as file:
                            #Opens this file^ Writes data here. 
                            writer = csv.writer(file, delimiter=",")
                            writer.writerow((FormXMissTime,FormCTDTime,Interpolation))
                        print("Interpolation: " + str(ShortInterp))

                    except KeyboardInterrupt:
                        print("Keyboard Interrupt")
                        breakIndicator = True
                        break    
                    except Exception as Exc:
                        ValidInterp = False
                        #print(Exc)
                        print("Not enough data to interpolate, common at startup.")
                        with open("CSDF.csv","a") as file:
                            #Opens this file^ Writes data here. 
                            writer = csv.writer(file, delimiter=",")
                            writer.writerow(("Not enough data","x","x","x","x",ValidInterp))
                continue
                   
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        breakIndicator = True
        exit()
    except Exception as Exc:
        print(Exc)
        print("An error occurred, data could not be recieved.")
        exit()
