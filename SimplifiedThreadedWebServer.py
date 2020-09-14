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

from ads1015 import ADS1015

PMInitBreak = Event()

XMissAbsent = False
CTDAbsent = False
PMAbsent = False

breakIndicator = False

define("port", default=8888, help="run on the given port", type=int)
clients = []

qXMiss = queue.Queue()
qCTD = queue.Queue()
qPwrM = queue.Queue()
qPlotsData = queue.Queue()
qWebSock = queue.Queue()
qPlots = queue.Queue()
qVoltages = queue.Queue()

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
XMissDataFile = str(Date) + "XMissRawData.csv"
XMissInterpFile = str(Date) + "XMissInterp.csv"
CTDDataFile = str(Date) + "CTDRawData.csv"
CTDInterpFile = str(Date) + "CTDInterp.csv"
PMDataFile = str(Date) + "PMRawData.csv"
PMInterpFile = str(Date) + "PMInterp.csv"
AllDataFile = str(Date) + "AllDataFile.csv"
VoltageLog = str(Date) + "VoltageLog.csv"

csvWrite(AllDataFile,("# Time","Absorbance Avg","Depth Avg","Temperature Avg","Power Avg","Exp Avg Power"),"a")
#csvWrite(VoltageLog,("# Time","Battery 1 Voltage","Battery 2 Voltage","12V Output Voltage","5V Output Voltage"), "a")

try:
    Serial1 = serial.Serial(PortChannel1)
    Serial1.baudrate = 19200
    Serial1.timeout = 1
    
    #Formatting CSV Files for XMiss if it is present.
    csvWrite(XMissDataFile,("# XMiss Sample Time","Absorbance"),"a")
    #csvWrite(XMissInterpFile, ("# Time Base Sample Time","XMiss Sample Time","Absorbance","Absorbance Interpolation")
except Exception as Exc:
    print(Exc)
    XMissAbsent = True
    print("Not connected to Channel1 on the Pi. Port is either in use or does not exist.")
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
            RawTime = time.time()
            LongTime = time.ctime(float(RawTime))
            FormatTime = (LongTime[11:19] + "." + str(RawTime)[12])
            global breakIndicator
            if breakIndicator == True:
                return
            try:
                SerialBytes = Serial1.readline()
                DecodedBytes = SerialBytes[0:len(SerialBytes)-2].decode("utf-8")
                CompressedBytes = re.sub("\s+"," ",DecodedBytes)
                StringValues = (CompressedBytes + " " + str(RawTime))
                Name,XMiss1,XMiss2,XMiss3,Absorbance,Wavelength,TimeStamp = StringValues.split(" ")
                qXMiss.put(TimeStamp + "," + Absorbance + "," + FormatTime)
                csvWrite(XMissDataFile,(FormatTime,Absorbance),"a")
            except:
                print("Could not retrieve data from XMiss.")
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
        print("CTD already running.")
        
    #Formatting CSV Files for CTD if it is present.
    csvWrite(CTDDataFile,("# CTD Sample Time","Time Since Started CTD","Depth","Temperature"),"a")
    #csvWrite(CTDInterpFile,("# Time Base Sample Time","CTD Sample Time","Depth","CTD Interpolation","Temperature"),"a")
    
except Exception as Exc:
    print(Exc)
    CTDAbsent = True
    print("Not connected to Channel2 on the Pi. Port is either in use or does not exist.")
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
            RawTime = time.time()
            LongTime = time.ctime(float(RawTime))
            FormatTime = (LongTime[11:19] + "." + str(RawTime)[12])
            global breakIndicator
            if breakIndicator == True:
                return
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
                StringValues = (CompressedBytes + ", " + str(RawTime))
                HashTemperature,Depth,CTDDate,CTDTime,TimeStamp = StringValues.split(", ")
                Temperature = HashTemperature[-len(HashTemperature)+2:-1]
                qCTD.put(TimeStamp + "," + Temperature + "," + Depth + "," + FormatTime)
                csvWrite(CTDDataFile,(FormatTime,CTDTime,Depth,Temperature),"a") 
            except Exception as Exc:
                print("Could not retrieve data from CTD.")
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
        #csvWrite(PMInterpFile,("# Time Base Sample Time","PwrM Sample Time","Power","PwrM Interpolation"),"a")
    except Exception as Exc:
        print(Exc)
        PMAbsent = True
        print("Not connected to Power Meter.")

class PMThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(PMThread,self).__init__()
        self.target = target
        self.name = name
        
    def run(self):
        while True:
            RawTime = time.time()
            LongTime = time.ctime(float(RawTime))
            FormatTime = (LongTime[11:19] + "." + str(RawTime)[12:13])
            global breakIndicator
            if breakIndicator == True:
                print('Caught interrupt, closing PM100')
                return
            try:
                Power = PowerMeter.read
                qPwrM.put(str(RawTime) + "," + str(Power) + "," + FormatTime)
                csvWrite(PMDataFile,(FormatTime,Power),"a")
                time.sleep(1)
            except Exception as Exc:
                print("Could not retrieve data from Power Meter. Killing PM thread.")
                time.sleep(0.1)
                return
        return

breakIndicator = False

class VoltageThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(VoltageThread,self).__init__()
        self.target = target
        self.name = name
        
    def run(self):
        a0cal = 4.108
        a1cal = 4.123
        a2cal = 4.095
        a3cal = 4.126

        ads = ADS1015()

        ads.set_mode('single')
        ads.set_programmable_gain(4.096)
        while True:
            global breakIndicator
            if breakIndicator == True:
                return
            try:
                TimeStamp = time.time()

                a0 = ads.get_voltage('in0/gnd')
                a1 = ads.get_voltage('in1/gnd')
                a2 = ads.get_voltage('in2/gnd')
                a3 = ads.get_voltage('in3/gnd')

                battery1Voltage = round(a0*a0cal, 2)
                battery2Voltage = round(a1*a1cal, 2)
                voltage12v = round(a2*a2cal, 2)
                voltage5v = round(a3*a3cal, 2)

                OutgoingVoltages = ("volt#" + str(battery1Voltage) + ", " + str(battery2Voltage) + ", " + str(voltage12v) + ", " + str(voltage5v))
                qVoltages.put(OutgoingVoltages)
                
                #csvWrite(VoltageLog,(TimeStamp,battery1Voltage,battery2Voltage,voltage12v,voltage5v),"a")
                
                time.sleep(10)

            except Exception as Exc:
                print("Could not retrieve voltages.")
                time.sleep(0.1)
                return
        return

class ConsumerWhileLoop(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(ConsumerWhileLoop,self).__init__()
        self.target = target
        self.name = name
        
    def run(self):
        #Consumer While Loop
        nextTime = time.time()
        while True:
            XMissTimeout = True
            CTDTimeout = True
            PMTimeout = True

            global breakIndicator
            if breakIndicator == True:
                return

            try:
                TimeBase = time.time()
                
                if qXMiss.empty() and qCTD.empty() and qPwrM.empty():
                    time.sleep(0.125)
                    print("Queues are empty.")
                    continue

                avgXMiss = 0
                countXMiss = 0
                while not qXMiss.empty():
                    countXMiss += 1
                    DataXMiss = qXMiss.get()
                    XMissTimeout = False
                    TimeStampXMiss,Absorbance,FormXMissTime = DataXMiss.split(",")
                    avgXMiss = float(avgXMiss) + float(Absorbance)
                if XMissTimeout == False:
                    ShortXMissInterp = avgXMiss / countXMiss

                avgDepth = 0
                countDepth = 0
                avgTemp = 0
                countTemp = 0
                while not qCTD.empty():
                    countDepth += 1
                    countTemp += 1
                    DataCTD = qCTD.get()
                    CTDTimeout = False
                    TimeStampCTD,Temperature,Depth,FormCTDTime = DataCTD.split(",")
                    avgCTD = float(avgDepth) + float(Depth)
                    avgTemp = float(avgTemp) + float(Temperature)
                if CTDTimeout == False:
                    ShortCTDInterp = avgDepth / countDepth
                    ShortTempInterp = avgTemp / countTemp

                avgPM = 0
                countPM = 0
                while not qPwrM.empty():
                    countPM += 1
                    DataPwrM = qPwrM.get()
                    PMTimeout = False
                    TimeStampPwrM,Power,FormPwrMTime = DataPwrM.split(",")
                    FormPower = '{:.2e}'.format(float(Power))
                    avgPM = float(avgPM) + float(FormPower)
                if PMTimeout == False:
                    ShortPMInterp = avgPM / countPM
                    try:
                        global weightConstant
                        pwrExpAvg = '{:.2e}'.format(weightConstant * ShortPMInterp + (1 - weightConstant) * float(lastPwrExpAvg))
                        lastPwrExpAvg = pwrExpAvg
                    except Exception as Exc:
                        pwrExpAvg = ShortPMInterp
                        lastPwrExpAvg = pwrExpAvg

                if XMissTimeout == True:
                    ShortXMissInterp = "NaN"
                if CTDTimeout == True:
                    ShortCTDInterp = "NaN"
                    ShortTempInterp = "NaN"
                if PMTimeout == True:
                    ShortPMInterp = "NaN"
                    pwrExpAvg = "NaN"
                
                OutgoingData = ("data#" + str(TimeBase) + ", " + str(ShortXMissInterp) + ", " + str(ShortCTDInterp) + ", " + str(ShortPMInterp) + ", " + str(pwrExpAvg))
                qWebSock.put(OutgoingData)

                csvWrite(AllDataFile,(TimeBase,ShortXMissInterp,ShortCTDInterp,ShortTempInterp,ShortPMInterp,pwrExpAvg),"a")
             
                OutgoingPlotData = (str(TimeBase) + ", " + str(ShortXMissInterp) + ", " + str(ShortCTDInterp) + ", " + str(ShortPMInterp) + ", " + str(pwrExpAvg))
                qPlotsData.put(OutgoingPlotData)
             
                nextTime += 2
                time.sleep(max(0, nextTime - time.time()))

            except Exception as Exc:
                print(Exc)

            except KeyboardInterrupt:
                print("Keyboard Interrupt")
                return
        return

rcParams.update({'figure.autolayout': True})

class PlottingThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,args=(), kwargs=None, verbose=None):
        super(PlottingThread,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        timeValues = []
        absorbanceValues = []
        depthValues = []
        powerValues = []
        pwrExpAvgValues = []
        while True:
            global breakIndicator
            if breakIndicator == True:
                return
                                    
            if qPlotsData.empty():
                time.sleep(0.125)
                continue

            try:
                if not qPlotsData.empty():
                    plotDataString = qPlotsData.get()
                    TimeStamp,AbsorbAvg,DepthAvg,PowerAvg,ExpAvgPower = plotDataString.split(", ")
                    timeValues.append(TimeStamp)
                    absorbanceValues.append(float(AbsorbAvg))
                    depthValues.append(float(DepthAvg))
                    powerValues.append(float(PowerAvg))
                    pwrExpAvgValues.append(float(ExpAvgPower))

                global plotLength
                localPlotLength = int(plotLength)

                timePlotValues = timeValues[-localPlotLength:-1]
                absorbancePlotValues = absorbanceValues[-localPlotLength:-1]
                depthPlotValues = depthValues[-localPlotLength:-1]
                powerPlotValues = powerValues[-localPlotLength:-1]
                pwrExpAvgPlotValues = pwrExpAvgValues[-localPlotLength:-1]

                #Data Plot        
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

            except Exception as Exc:
                print(Exc)
                print("Data Graph could not be generated.")
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
        global breakIndicator
        if breakIndicator == True:
            return
        
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
                EncodedSensorData = SensorData.encode()
                for c in clients:
                    c.write_message(EncodedSensorData)
            if not qPlots.empty():
                PlotData = qPlots.get()
                for c in clients:
                    c.write_message(PlotData)
            if not qVoltages.empty():
                VoltageData = qVoltages.get()
                EncodedVoltageData = VoltageData.encode()
                for c in clients:
                    c.write_message(EncodedVoltageData)
            global breakIndicator
            if breakIndicator == True:
                return
        
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

    VoltageThread().start()

    ConsumerWhileLoop().start()
    
    PlottingThread().start()

    WebSocketHost().start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            breakIndicator = True
            exit()
