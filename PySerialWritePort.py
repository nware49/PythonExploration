import serial
#Attempts to open and assign a serial port
#If it cannot open the port, it will print an error message
try:
    ser1 = serial.Serial("COM3")
except:
    print("Could not open port. Port is either in use or does not exist.")

print(ser1.name)
#Funky - need the b before writing something to a string
ser1.write(b'Connected to COM3')

#This program successfully writes a message to the serial port
#The message pops up on the KiTTY monitor for port 4
#Since the wire connects COM3 to COM4, the message is sent from COM3 to COM4
#Implemented successful Exception Handling to test for port availability
