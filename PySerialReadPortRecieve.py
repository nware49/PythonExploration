import serial
import time
Port2 = "COM4"  #This is which port the data will be recieved at, can be changed
#Attempts to open and assign a serial port
#If it cannot open the port, it will print an error message
try:
    ser2 = serial.Serial(Port2)
except:
    print("Could not open port. Port is either in use or does not exist.")
print(ser2.name)
while 1:
  tdata = ser2.read()           # Wait forever for anything
  time.sleep(1)                 # Sleep (or inWaiting() doesn't give the correct value)
  data_left = ser2.inWaiting()  # Get the number of characters ready to be read
  tdata += ser2.read(data_left) # Do the read and combine it with the first character
  print(tdata)                  # Print the sent data to the screen

#This program successfully recieves a message from the serial port
#The program will run continuously, and can recieve data until it is closed
#Implemented successful Exception Handling to test for port availability
#The program is complemeted by "PySerialWritePortSend"
#This is essentially part 2
