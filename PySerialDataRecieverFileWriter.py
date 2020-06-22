import serial
import time
import csv
Port2 = "COM4"  #This is which port the data will be recieved at, can be changed
#Attempts to open and assign a serial port
#If it cannot open the port, it will print an error message
try:
    ser2 = serial.Serial(Port2)
except:
    print("Could not open port. Port is either in use or does not exist.")
    exit()
print(ser2.name)

while True:
    try:
        ser_bytes = ser2.readline()
        DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
        DecodedBytes = DecodedBytes
        print(DecodedBytes)
        IncrementNum,TimeVal,RandFloat,SineVal,CosineVal = DecodedBytes.split(",")
        
        with open("DataSimulation.csv","a") as file:
            #Opens this file^ Writes data here. 
            writer = csv.writer(file, delimiter=",")
            writer.writerow((IncrementNum,TimeVal,RandFloat,SineVal,CosineVal))
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        exit()
    except:
        print("An error occurred, data could not be recieved.")
        exit()
