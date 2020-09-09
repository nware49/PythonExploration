#!/usr/bin/python3

import time
import random
import serial
import csv
import re
import threading
from threading import Thread, Event, Lock
import collections
import queue
import logging
from datetime import datetime
import base64
import numpy as np
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import ticker
from matplotlib import rcParams
import usbtmc as usbtmc
import usb
from usb.core import find as finddev
from ThorlabsPM100 import ThorlabsPM100

import distutils.util

import tornado.httpserver
import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.gen
from tornado.options import define, options
import io
import asyncio

PMInitBreak = Event()

CTDAbsent = False
PMAbsent = False

breakIndicator = False

define("port", default=8888, help="run on the given port", type=int)
clients = []

timeValues = []
absorbanceValues = []
depthValues = []
powerValues = []
pwrExpAvgValues = []

qXMiss = queue.Queue(10)
qCTD = queue.Queue(10)
qPwrM = queue.Queue(10)
qWebSock = queue.Queue(10)
qPlots = queue.Queue(10)

plotLengthLock = threading.Lock()
plotLength = 100
visiblePlotsLock = threading.Lock()
absorbanceVisible = True
depthVisible = True
powerVisible = True
pwrExpAvgVisible = True
xAxisTypeLock = threading.Lock()
xAxisType = "Time"

weightConstant = 0.2

lastPwrExpAvg = None

PortChannel1 = "/dev/ttySC0"
PortChannel2 = "/dev/ttySC1"

def csvWrite(FileName,DataLine,EditMethod): #Ex. csvWrite("Data.csv",(Time,Distance,Velocity),"a")
    with open(FileName,EditMethod) as file: 
        writer = csv.writer(file, delimiter=",")
        writer.writerow(DataLine)

now = datetime.now()
Date = now.strftime("%Y%m%d-%H%M_")

#File Names
XMissDataFile = str(Date) + "XMissData.csv"
CTDDataFile = str(Date) + "CTDData.csv"
CTDInterpFile = str(Date) + "CTDInterp.csv"
PMDataFile = str(Date) + "PMData.csv"
PMInterpFile = str(Date) + "PMInterp.csv"
AllDataFile = str(Date) + "AllDataFile.csv"

csvWrite(AllDataFile,("# Time", "Absorbance", "Depth", "Power", "Exp Avg Power"),"a")

try:
    Serial1 = serial.Serial(PortChannel1)
    Serial1.baudrate = 19200
    Serial1.timeout = 1
    
    #Formatting CSV Files for XMiss if it is present.
    csvWrite(XMissDataFile,("# XMiss Sample Time","Absorbance"),"a")
except:
    print("Not connected to Channel1. Port is either in use or does not exist.")
    exit()

class XMissThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(XMissThread,self).__init__()
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
                    #print("Could not retrieve data from XMiss.")
                    time.sleep(0.1)
                    continue
        return

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
        print("CTD init not used. Either absent or already running.")
        
    #Formatting CSV Files for CTD if it is present.
    csvWrite(CTDDataFile,("# CTD Sample Time","Temperature"),"a")
    csvWrite(CTDInterpFile,("# XMiss Sample Time","CTD Sample Time","CTD Interpolation"),"a")
    
except Exception as Exc:
    print(Exc)
    print("Not connected to Channel2. Port is either in use or does not exist.")
    exit()

class CTDThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(CTDThread,self).__init__()
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
                    qCTD.put(TimeStamp + "," + Temperature + "," + Depth)
                except Exception as Exc:
                    #print("Could not retrieve data from CTD.")
                    time.sleep(0.1)
                    continue
        return

def PowerMeterConfig():
    try:
        dev = finddev(idVendor=0x1313, idProduct=0x807b)
        dev.reset()

        print('Initializing PM100... ', end='')
        instr =  usbtmc.Instrument(4883, 32891)
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

class PMThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(PMThread,self).__init__()
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

breakIndicator = False

rcParams.update({'figure.autolayout': True})

class ConsumerWhileLoop(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(ConsumerWhileLoop,self).__init__()
        self.target = target
        self.name = name
        
    def run(self):
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
            XMissTimeout = True
            CTDTimeout = True
            PwrMTimeout = True
            XMissValInterp = False
            CTDValInterp = False
            PMValInterp = False

            global breakIndicator
            if breakIndicator == True:
                break

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
                
                try:
                    DataCTD = qCTD.get(timeout=2)
                    CTDTimeout = False
                except KeyboardInterrupt:
                    print("Keyboard Interrupt")
                    breakIndicator = True
                    exit()
                except:
                    ShortCTDInterp = "-"
                    
                if CTDTimeout == False:
                    TimeStampCTD,Temperature,Depth = DataCTD.split(",")
                    LongCTDTime = time.ctime(float(TimeStampCTD))
                    FormCTDTime = LongCTDTime[11:19]
                    IDACTDTime.append(float(TimeStampCTD))
                    IDACTD.append(float(Depth))
                    IDAFormCTDTime.append(str(FormCTDTime))
                    csvWrite(CTDDataFile,(FormCTDTime,Depth),"a")                
                c = -1
                XMissTimeout = True
                while not qXMiss.empty():
                    XMissTimeout = False
                    #Provides Master Timeline, therefore interpolation is unnecessary for XMiss.
                    c += 1
                    DataXMiss = qXMiss.get()
                    TimeStampXMiss,Absorbance = DataXMiss.split(",")
                    LongXMissTime = time.ctime(float(TimeStampXMiss))
                    FormXMissTime = LongXMissTime[11:19]
                    IDAXMissTime.append(float(TimeStampXMiss))
                    IDAXMiss.append(float(Absorbance))
                    IDAFormXMissTime.append(str(FormXMissTime))
                    csvWrite(XMissDataFile,(FormXMissTime,Absorbance),"a")
                    if CTDTimeout == False:
                        try:
                            CTDInterpolation = (IDACTD[-2]) + (((IDACTD[-1]) - (IDACTD[-2])) /
                                ((IDACTDTime[-1]) - (IDACTDTime[-2]))) *  ((IDAXMissTime[-1]) - (IDACTDTime[-2]))
                            ShortCTDInterp = format(CTDInterpolation, '.3f')
                            csvWrite(CTDInterpFile,(FormXMissTime,FormCTDTime,ShortCTDInterp),"a")
                            CTDValInterp = True
                        except KeyboardInterrupt:
                            print("Keyboard Interrupt")
                            breakIndicator = True
                            exit()
                        except Exception as Exc:
                            csvWrite(CTDInterpFile,("Not enough data",),"a")
                            CTDValInterp = False
                            ShortCTDInterp = Depth

                while not qPwrM.empty():
                    DataPwrM = qPwrM.get()
                    PwrMTimeout = False
                    TimeStampPwrM,Power = DataPwrM.split(",")
                    FormPower = '{:.2e}'.format(float(Power))
                    LongPwrMTime = time.ctime(float(TimeStampPwrM))
                    FormPwrMTime = LongPwrMTime[11:19]
                    IDAFormPwrMTime.append(str(FormPwrMTime))
                    IDAPwrMTime.append(float(TimeStampPwrM))
                    IDAPwr.append(float(Power))
                    csvWrite(PMDataFile,(FormPwrMTime,FormPower),"a")
                    if XMissTimeout == False:
                        try:
                            XMissTime = IDAXMissTime[-1 - c]
                            FormXMissTime = time.ctime(float(XMissTime))
                            PMInterpolation = (IDAPwr[-2]) + (((IDAPwr[-1]) - (IDAPwr[-2])) /
                                ((IDAPwrMTime[-1]) - (IDAPwrMTime[-2]))) *  ((XMissTime) - (IDAPwrMTime[-2]))
                            ShortPMInterp = float('{:.2e}'.format(PMInterpolation))
                            csvWrite(PMInterpFile,(FormXMissTime,FormPwrMTime,FormPower,ShortPMInterp),"a")
                            PMValInterp = True
                            global weightConstant
                            pwrExpAvg = '{:.2e}'.format(weightConstant*ShortPMInterp + (1 - weightConstant)*float(lastPwrExpAvg))
                            lastPwrExpAvg = pwrExpAvg
                            
                        except KeyboardInterrupt:
                            print("Keyboard Interrupt")
                            breakIndicator = True
                            exit()
                        except Exception as Exc:
                            csvWrite(PMInterpFile,("Not enough data",),"a")
                            PMValInterp = False
                            ShortPMInterp = FormPower
                            pwrExpAvg = ShortPMInterp
                            lastPwrExpAvg = pwrExpAvg
                        c -= 1
                        
                CurrentTime = datetime.now()
                PlotTime = format((time.time()), '.1f')
                StrfTime = CurrentTime.strftime("%H:%M:%S")
                if XMissTimeout == True:
                    Absorbance = "-"
                    if CTDTimeout == False:
                        ShortCTDInterp = Depth
                    if PwrMTimeout == False:
                        ShortPMInterp = FormPower
                        pwrExpAvg = ShortPMInterp
                if PwrMTimeout == True:
                    ShortPMInterp = "-"
                    pwrExpAvg = "-"

                OutgoingData = ("data#" + str(StrfTime) + ", " + str(Absorbance) + ", " + str(ShortCTDInterp) + ", " + str(ShortPMInterp) + ", " + str(pwrExpAvg))
                qWebSock.put(OutgoingData)

                csvWrite(AllDataFile,(StrfTime,Absorbance,ShortCTDInterp,ShortPMInterp,pwrExpAvg),"a")

                if XMissTimeout == True or PwrMTimeout == True or CTDTimeout == True:
                    continue
                
                try:
                    pwrExpAvgValues.append(float(pwrExpAvg))
                    powerValues.append(float(ShortPMInterp))
                    depthValues.append(float(ShortCTDInterp))
                    absorbanceValues.append(float(Absorbance))
                    timeValues.append(CurrentTime)
                    
                except Exception as Exc:
                    print(Exc)
                    print("Missing Data")
                    continue
                    
                historyLength = 1000
                if len(timeValues)>historyLength:
                    timeValues.pop(0)
                if len(absorbanceValues)>historyLength:
                    absorbanceValues.pop(0)
                if len(depthValues)>historyLength:
                    depthValues.pop(0)
                if len(powerValues)>historyLength:
                    powerValues.pop(0)
                if len(pwrExpAvgValues)>historyLength:
                    pwrExpAvgValues.pop(0)
                
                global plotLength
                localPlotLength = int(plotLength)
                
                timePlotValues = timeValues[-localPlotLength:-1]
                absorbancePlotValues = absorbanceValues[-localPlotLength:-1]
                depthPlotValues = depthValues[-localPlotLength:-1]
                powerPlotValues = powerValues[-localPlotLength:-1]
                pwrExpAvgPlotValues = pwrExpAvgValues[-localPlotLength:-1]
                
                #Data Plot
                try:        
                    figPower = plt.gcf()
                    plt.clf()
                    figPower, ax1 = plt.subplots()

                    color1 = "tab:green"
                    color2 = "tab:red"
                    color3 = "tab:blue"
                    color4 = "tab:purple"
                    color5 = "tab:orange"

                    global absorbanceVisible
                    global depthVisible
                    global powerVisible
                    global pwrExpAvgVisible

                    localAbsorbanceVisible = absorbanceVisible
                    localDepthVisible = depthVisible
                    localPowerVisible = powerVisible
                    localPwrExpAvgVisible = pwrExpAvgVisible

                    global xAxisType
                    xAxisTypeLocal = xAxisType
                    if xAxisTypeLocal == "Time":
                        xAxisValues = timePlotValues
                        ax1.set_xlabel("time", color=color1)
                    if xAxisTypeLocal == "Depth":
                        xAxisValues = depthPlotValues
                        ax1.set_xlabel("depth", color=color1)
                        
                    try:
                        ax1.margins(y=0.5)
                        if localPwrExpAvgVisible == True:
                            ax1.plot(xAxisValues, pwrExpAvgPlotValues, marker= "o", color= color2, label="Exp Avg Power")
                        if localPowerVisible == True:
                            ax1.plot(xAxisValues, powerPlotValues, marker= "o", color= color4, label="Power")
                        for label in ax1.get_xticklabels():
                            label.set_rotation(30)
                            label.set_ha('right')
                        ax1.set_ylabel("power (nanowatts * m^2)", color=color4)
                        ax1.tick_params(axis='y', which='both', labelcolor=color4)
                        ax1.tick_params(axis='x', labelcolor=color1)
                        ax1.set_yscale('log')
                        ax1.legend(loc = 'upper left')
                    
                    except:
                        ax1.set_ylabel("Power Meter Absent", color=color4)

                    if localAbsorbanceVisible == True:
                        try:
                            ax2 = ax1.twinx()
                            ax2.margins(x=None, y=1.5)
                            ax2.plot(xAxisValues, absorbancePlotValues, marker= "o", color= color3, label="Absorbance")
                            ax2.set_ylabel("absorbance (nm/m)", labelpad=12, color=color3)
                            ax2.tick_params(axis='y', labelcolor=color3)
                            ax2.legend(loc = 'upper right')
                            
                        except:
                            ax2.set_ylabel("Transmissometer Absent", color=color3)

                    if localDepthVisible == True and not xAxisTypeLocal == "Depth":
                        try:
                            ax3 = ax1.twinx()
                            ax3.margins(x=None, y=1.5)
                            ax3.plot(timePlotValues, depthPlotValues, marker= "o", color= color5, label="Depth")
                            ax3.set_ylabel("depth (m)", color=color5)
                            ax3.tick_params(axis='y', labelcolor=color5)
                            ax3.legend(loc = 'lower left')
                            
                        except:
                            ax3.set_ylabel("CTD Absent", color=color5)
                    
                    if xAxisTypeLocal == "Time":
                        plt.title("Power and Absorbance and Depth vs. Time")
                    if xAxisTypeLocal == "Depth":
                        plt.title("Power and Absorbance vs. Depth")
                    #plt.figlegend()
                    figPower.savefig("PowerPlot.jpg")
                    figPower.tight_layout()
                    plt.close()

                    plot = open("PowerPlot.jpg", 'rb')
                    plotRead = plot.read()
                    plotEncode = base64.encodebytes(plotRead)
                    qPlots.put(plotEncode)
                   
                except Exception as Exc:
                    print(Exc)
                    print("Raw Data Graph could not be generated.")
                        
            except Exception as Exc:
                print(Exc)

            except KeyboardInterrupt:
                print("Keyboard Interrupt")
                return

class IndexHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('WebsiteLayout.html', filename='PowerPlot.jpg')
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True
    def open(self):
        print('new connection')
        clients.append(self)
    def on_message(self, message):
        MessageType = message[0:4]
        if MessageType == "len#":
            print("New Plot Length: " + message[4:])
            plotLengthLock.acquire()
            global plotLength
            plotLength = message[4:]
            plotLengthLock.release()
        if MessageType == "vis#":
            plotsString = message[4:]
            print("Plot Change: " + plotsString)
            visiblePlots = plotsString.split(", ")
            global absorbanceVisible
            absorbanceVisible = distutils.util.strtobool(visiblePlots[0])
            global depthVisible
            depthVisible = distutils.util.strtobool(visiblePlots[1])
            global powerVisible
            powerVisible = distutils.util.strtobool(visiblePlots[2])
            global pwrExpAvgVisible
            pwrExpAvgVisible = distutils.util.strtobool(visiblePlots[3])
        if MessageType == "axs#":
            axisTypeString = message[4:]
            global xAxisType
            xAxisType = axisTypeString
        
    def on_close(self):
        print('connection closed')
        clients.remove(self)

class WebSocketHost(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(WebSocketHost,self).__init__()
        self.target = target
        self.name = name
        
    def run(self):
        def PushToClients():
            if not qWebSock.empty():
                SensorData = qWebSock.get()
                EncodedData = SensorData.encode()
                for c in clients:
                    c.write_message(EncodedData)
            if not qPlots.empty():
                PlotData = qPlots.get()
                for c in clients:
                    c.write_message(PlotData)
        
        asyncio.set_event_loop(asyncio.new_event_loop())
        tornado.options.parse_command_line()
        app = tornado.web.Application(
            handlers=[
                (r"/", IndexHandler),
                (r"/ws", WebSocketHandler)
            ]
        )
        httpServer = tornado.httpserver.HTTPServer(app)
        httpServer.listen(options.port)
        print("Listening on port:" + str(options.port))        
        webSocketLoop = tornado.ioloop.IOLoop.current()
        callbackScheduler = tornado.ioloop.PeriodicCallback(PushToClients,100)
        callbackScheduler.start()
        webSocketLoop.start()
            
if __name__ == '__main__':

    PowerMeterInit = Thread(target=PowerMeterConfig)
    PowerMeterInit.start()
    PowerMeterInit.join(timeout=5)

    PMInitBreak.set()

    XMissThrd = XMissThread()
    CTDThrd = CTDThread()
    PMThrd = PMThread()

    XMissThrd.start()
    CTDThrd.start()
    if PMAbsent == False:
        PMThrd.start()

    ConsumerThread = ConsumerWhileLoop()
    ConsumerThread.start()

    WebSockHost = WebSocketHost()
    WebSockHost.start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            breakIndicator = True
            exit()
