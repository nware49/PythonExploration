import serial
import time
import random
i = 0

#This text will be sent across the serial ports
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
    Time = time.time()
    x = random.uniform(0,10)
    y = random.randint(0,100)
    z = random.randint(100,500)
    IncrementNum = bytes(str(i),"ascii")
    TimeVar = bytes(str(Time),"ascii")
    RandVar3 = bytes(str(y),"ascii")
    FloatNum = bytes(str(x),"ascii")
    RandVar5 = bytes(str(z),"ascii")
    LongString = bytes(str((IncrementNum,TimeVar,RandVar3,FloatNum,RandVar5)),"ascii")
    try:
        ser1.write(LongString)
        time.sleep(5)
    except KeyboardInterrupt:
        quit()
        print("Keyboard Interrupt")
    except:
        print("Cannot write to port")
        quit()
