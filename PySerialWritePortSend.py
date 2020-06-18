import serial
Text = b"Connected to COM3!" #This text will be sent across the serial ports
Port1 = "COM3" #This is which port the data will be sent from
#Attempts to open and assign a serial port
#If it cannot open the port, it will print an error message
try:
    ser1 = serial.Serial(Port1)
except:
    print("Could not open port. Port is either in use or does not exist.")

print(ser1.name)
#Funky Python - need the b before writing something to a string
#Learned that the b converts the text to binary
ser1.write(Text)

#This program successfully writes a message to the serial port
#The message pops up on the KiTTY monitor for port 4
#Since the wire connects COM3 to COM4, the message is sent from COM3 to COM4
#Implemented successful Exception Handling to test for port availability
#This program will successfully send a message from one serial port to another
#The program is complemented by "PySerialReadPortRecieve"
#This is essentially part 1
