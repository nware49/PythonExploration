import serial
import math
import time
import threading
import random
from datetime import datetime

Port1 = "COM3" #This is which port the data will be sent from
Port2 = "COM4"
#Attempts to open and assign a serial port
#If it cannot open the port, it will print an error message
try:
    ser1 = serial.Serial(Port1)
    ser2 = serial.Serial(Port2)
except:
    print("Could not open ports. Port is either in use or does not exist.")
    exit()
print(ser1.name)
print(ser2.name)

breakIndicator = 0

class Serial1Write(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(Serial1Write,self).__init__()

    def run(self):
        a = -1.1
        i = -1
        pi = math.pi
        while True:
            if breakIndicator == 1:
                break
            i += 1
            pi = math.pi
            curDT = datetime.now()
            Time = curDT.strftime("%H:%M:%S")
            x = random.uniform(0,10)
            RandFloat = format(x, '.5f')
            y = math.sin((i*pi)/12)
            SineWave = format(y, '.4f')
            z = math.cos((i*pi)/12)
            CosineWave = format(z, '.4f')
            LongString = (str(i) + "," + str(Time) + "," + str(RandFloat) + "," + str(SineWave) + "," + str(CosineWave) + "\n")
            try:
                ser1.write(LongString.encode())
                print (LongString)
                time.sleep(1)
            except:
                print("Cannot write to port.")
                break
        return
        
class Serial2Write(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
        super(Serial2Write,self).__init__()
        
    def run(self):
        a = -1.1
        i = -1
        pi = math.pi
        while True:
            if breakIndicator == 1:
                break
            i += 1
            a += 1.1
            aFormat = format(a, '.1f')
            curDT = datetime.now()
            Time = curDT.strftime("%H:%M:%S")
            x = random.uniform(0,10)
            RandFloat = format(x, '.5f')
            y = math.sin((i*pi)/12)
            SineWave = format(y, '.4f')
            z = math.cos((i*pi)/12)
            CosineWave = format(z, '.4f')
            LongString = (str(aFormat) + "," + str(Time) + "," + str(RandFloat) + "," + str(SineWave) + "," + str(CosineWave) + "\n")
            try:
                ser2.write(LongString.encode())
                print (LongString)
                time.sleep(1.1)
            except:
                print("Cannot write to port.")
                break
        return
if __name__ == '__main__':
        writer1 = Serial1Write()
        writer2 = Serial2Write()
        
        writer1.start()
        writer2.start()
try:           
    while True:
        continue
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    breakIndicator = 1
    exit()
except:
    print("Could not spawn threads to begin writing to ports.")
    exit()
