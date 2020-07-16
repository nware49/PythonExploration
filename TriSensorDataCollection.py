import threading
from threading import Thread, Event
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
from ThorlabsPM100 import ThorlabsPM100
import usbtmc as usbtmc
import usb
import time
from usb.core import find as finddev

CTDAbsent = False
PMInitBreak = Event()
PMAbsent = False

PortChannel1 = "/dev/ttySC0"
PortChannel2 = "/dev/ttySC1"

def csvWrite(FileName,DataLine,EditMethod): #Ex. csvWrite("Data.csv",(Time,Distance,Velocity),"a")
    with open(FileName,EditMethod) as file: 
        writer = csv.writer(file, delimiter=",")
        writer.writerow(DataLine)

now = datetime.now()
Date = now.strftime("%m%d%Y")
print(Date)

#File Names
XMissDataFile = str(Date) + "XMissData.csv"
CTDDataFile = str(Date) + "CTDData.csv"
CTDInterpFile = str(Date) + "CTDInterp.csv"
PMDataFile = str(Date) + "PMData.csv"
PMInterpFile = str(Date) + "PMInterp.csv"

try:
    Serial1 = serial.Serial(PortChannel1)
    Serial1.baudrate = 19200
    Serial1.timeout = 1
    
    #Formatting CSV Files for XMiss if it is present.
    csvWrite(XMissDataFile,("# XMiss Sample Time","Absorbance"),"a")
except:
    print("Not connected to Channel1. Port is either in use or does not exist.")
    exit()

try:
    Serial2 = serial.Serial(PortChannel2)
    Serial2.baudrate = 9600
    Serial2.timeout = 2

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
        
    #Formatting CSV Files for CTD if it is present.
    csvWrite(CTDDataFile,("# CTD Sample Time","Temperature"),"a")
    csvWrite(CTDInterpFile,("# XMiss Sample Time","CTD Sample Time","CTD Interpolation"),"a")
    
except Exception as Exc:
    print(Exc)
    print("Not connected to Channel2. Port is either in use or does not exist.")
    exit()

def PowerMeterConfig():
    try:
        dev = finddev(idVendor=0x1313, idProduct=0x8072)
        dev.reset()

        print('Initializing PM100... ', end='')
        instr =  usbtmc.Instrument(4883, 32882)
        global PowerMeter
        PowerMeter = ThorlabsPM100(inst=instr)
        
        try:
            print(instr.ask("*IDN?"))

        except usb.core.USBError:
            print('...Failed! Continuing to program without PM')
            global PMAbsent
            PMAbsent = True
            
        #Formatting CSV Files for Power Meter if it is present.
        csvWrite(PMDataFile,("# PwrM Sample Time","Power"),"a")
        csvWrite(PMInterpFile,("# XMiss Sample Time","PwrM Sample Time","Power","PwrM Interpolation"),"a")
    except Exception as Exc:
        print(Exc)
        print("Not connected to Power Meter.")
        PMAbsent = True

BufferSize = 10
qXMiss = queue.Queue(BufferSize)
qCTD = queue.Queue(BufferSize)
qPwrM = queue.Queue(BufferSize)
breakIndicator = False

class ProdThrdXMiss1(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(ProdThrdXMiss1,self).__init__()
        self.target = target
        self.name = name
        Serial1.flush()

    def run(self):
        Serial1.flush()
        while True:
            Time = time.time()
            if breakIndicator == True:
                break
            if not qXMiss.full():
                try:
                    SerialBytes = Serial1.readline()
                    DecodedBytes = SerialBytes[0:len(SerialBytes)-2].decode("utf-8")
                    CompressedBytes = re.sub("\s+"," ",DecodedBytes)
                    StringValues = (CompressedBytes + " " + str(Time))
                    Name,XMiss1,XMiss2,XMiss3,Absorbance,Wavelength,TimeStamp = StringValues.split(" ")
                    qXMiss.put(TimeStamp + "," + Absorbance)
                except:
                    print("Could not retrieve data from XMiss.")
                    time.sleep(0.1)
                    continue
        return

class ProdThrdCTD2(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(ProdThrdCTD2,self).__init__()
        self.target = target
        self.name = name
        Serial2.flush()

    def run(self):
        Serial2.flush()
        while True:
            Time = time.time()
            if breakIndicator == True:
                break
            if not qCTD.full():
                try:
                    SerialBytes = Serial2.readline()
                    if SerialBytes == b'\r\n':
                        continue
                    DecodedBytes = SerialBytes[0:len(SerialBytes)-2].decode("utf-8")
                    if DecodedBytes == "S>":
                        continue
                    if DecodedBytes == "S>startnow":
                        continue
                    CompressedBytes = re.sub("\s+"," ",DecodedBytes)
                    StringValues = (CompressedBytes + ", " + str(Time))
                    HashTemperature,Depth,CTDDate,CTDTime,TimeStamp = StringValues.split(", ")
                    Temperature = HashTemperature[-len(HashTemperature)+2:-1]
                    qCTD.put(TimeStamp + "," + Temperature)
                except Exception as Exc:
                    print("Could not retrieve data from CTD.")
                    time.sleep(0.1)
                    continue
        return

class ProdThrdPwrM3(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(ProdThrdPwrM3,self).__init__()
        self.target = target
        self.name = name
        
    def run(self):
        while True:
            if breakIndicator == True:
                print('Caught interrupt, closing PM100')
                break
            try:
                TimeStamp = time.time()
                if not qPwrM.full():
                    Power = PowerMeter.read
                    #print(PowerMeter.read)
                    qPwrM.put(str(TimeStamp) + "," + str(Power))
                time.sleep(1)
            except Exception as Exc:
                print("Could not retrieve data from Power Meter. Killing PM thread.")
                time.sleep(0.1)
                return
        return
        

try:
    if __name__ == '__main__':

        PowerMeterInit = Thread(target=PowerMeterConfig)

        PowerMeterInit.start()
        PowerMeterInit.join(timeout=5)

        PMInitBreak.set()
        
        pc1 = ProdThrdXMiss1(name='producerChannel1')
        pc2 = ProdThrdCTD2(name='producerChannel2')
        pc3 = ProdThrdPwrM3(name='producerChannel3')

        pc1.start()
        pc2.start()
        if PMAbsent == False:
            pc3.start()

except Exception as Exc:
    print(Exc)
    print("An error occured, the threads could not be spawned.")
    exit()

IDAFormXMissTime = collections.deque()
IDAXMissTime = collections.deque()
IDAXMiss = collections.deque()

IDAFormCTDTime = collections.deque()
IDACTDTime = collections.deque()
IDACTD = collections.deque()

IDAFormPwrMTime = collections.deque()
IDAPwrMTime = collections.deque()
IDAPwr = collections.deque()

#Consumer While Loop
while True:
    XMissTimeout = False
    CTDTimeout = False
    PwrMTimeout = False
    #if breakIndicator == True:
        #break

    if len(IDAXMissTime)>5:
        IDAXMissTime.popleft()
    if len(IDAXMiss)>5:
        IDAXMiss.popleft()
    if len(IDAFormXMissTime)>5:
        IDAFormXMissTime.popleft()
    if len(IDACTDTime)>5:
        IDACTDTime.popleft()
    if len(IDACTD)>5:
        IDACTD.popleft()
    if len(IDAFormCTDTime)>5:
        IDAFormCTDTime.popleft()
    if len(IDAFormPwrMTime)>5:
        IDAFormPwrMTime.popleft()
    if len(IDAPwrMTime)>5:
        IDAPwrMTime.popleft()
    if len(IDAPwr)>5:
        IDAPwr.popleft()

    try:
        
        if qXMiss.empty() and qCTD.empty() and qPwrM.empty():
            time.sleep(0.125)
            continue

        #Slowest Data Transmitter
        try:
            DataCTD = qCTD.get(timeout=2)
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            breakIndicator = True
            exit()
        except:
            CTDTimeout = True
            
        if CTDTimeout == False:
            TimeStampCTD,Temperature = DataCTD.split(",")
            FormCTDTime = time.ctime(float(TimeStampCTD))
            IDACTDTime.append(float(TimeStampCTD))
            IDACTD.append(float(Temperature))
            IDAFormCTDTime.append(str(FormCTDTime))
            csvWrite(CTDDataFile,(FormCTDTime,Temperature),"a")
        c = -1
        XMissTimeout = True
        while not qXMiss.empty():
            XMissTimeout = False
            #Provides Master Timeline, therefore interpolation is unnecessary for XMiss.
            c += 1
            DataXMiss = qXMiss.get()
            TimeStampXMiss,Absorbance = DataXMiss.split(",")
            FormXMissTime = time.ctime(float(TimeStampXMiss))
            IDAXMissTime.append(float(TimeStampXMiss))
            IDAXMiss.append(float(Absorbance))
            IDAFormXMissTime.append(str(FormXMissTime))
            csvWrite(XMissDataFile,(FormXMissTime,Absorbance),"a")
            if CTDTimeout == False:
                try:
                    Interpolation = (IDACTD[-2]) + (((IDACTD[-1]) - (IDACTD[-2])) / ((IDACTDTime[-1]) - (IDACTDTime[-2]))) *  ((IDAXMissTime[-1]) - (IDACTDTime[-2]))
                    ShortInterp = format(Interpolation, '.3f')
                    csvWrite(CTDInterpFile,(FormXMissTime,FormCTDTime,ShortInterp),"a")
                    print(ShortInterp)
                except KeyboardInterrupt:
                    print("Keyboard Interrupt")
                    breakIndicator = True
                    exit()
                except:
                    print("Not enough data to interpolate CTD, common at startup.")
                    csvWrite(CTDInterpFile,("Not enough data",),"a")
        while not qPwrM.empty():
            DataPwrM = qPwrM.get()
            TimeStampPwrM,Power = DataPwrM.split(",")
            FormPwrMTime = time.ctime(float(TimeStampPwrM))
            IDAFormPwrMTime.append(str(FormPwrMTime))
            IDAPwrMTime.append(float(TimeStampPwrM))
            IDAPwr.append(float(Power))
            csvWrite(PMDataFile,(FormPwrMTime,Power),"a")
            if XMissTimeout == False:
                try:
                    XMissTime = IDAXMissTime[-1 - c]
                    FormXMissTime = time.ctime(float(XMissTime))
                    Interpolation = (IDAPwr[-2]) + (((IDAPwr[-1]) - (IDAPwr[-2])) / ((IDAPwrMTime[-1]) - (IDAPwrMTime[-2]))) *  ((XMissTime) - (IDAPwrMTime[-2]))
                    #ShortInterp = format(Interpolation, '.3f')
                    csvWrite(PMInterpFile,(FormXMissTime,FormPwrMTime,Power,Interpolation),"a")
                    print(Interpolation)
                except KeyboardInterrupt:
                    print("Keyboard Interrupt")
                    breakIndicator = True
                    exit()
                except:
                    print("Not enough data to interpolate Power Meter, common at startup.")
                    csvWrite(PMInterpFile,("Not enough data",),"a")
            c -= 1
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        breakIndicator = True
        exit()
    except Exception as Exc:
        print(Exc)
        print("An error occurred, data could not be recieved.")
        exit()
