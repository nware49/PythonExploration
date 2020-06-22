import serial
import math
import time
import random
from datetime import datetime
i = 0
pi = math.pi
Port1 = "COM3" #This is which port the data will be sent from
#Attempts to open and assign a serial port
#If it cannot open the port, it will print an error message
try:
    ser1 = serial.Serial(Port1)
except:
    print("Could not open port. Port is either in use or does not exist.")
    exit()
print(ser1.name)

while True:
    i += 1
    curDT = datetime.now()
    Time = curDT.strftime("%H:%M:%S")
    x = random.uniform(0,10)
    RandFloat = format(x, '.5f')
    y = math.sin((i*pi)/12)
    SineWave = format(y, '.4f')
    z = math.cos((i*pi)/12)
    CosineWave = format(z, '.4f')
    LongString = (str(i) + "," + str(Time) + "," + str(RandFloat) + "," + str(SineWave) + "," + str(CosineWave) + "\r\n")
    print (LongString)
    try:
        ser1.write(LongString.encode())
        time.sleep(5)
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        quit()
    except:
        print("Cannot write to port.")
        quit()
