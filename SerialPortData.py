import serial
#basic edgeport program to register serial ports and read
#known error. can't send strings thru serial ports
#learned ser.write determines how many bytes of data are written to ports
with serial.Serial('COM3') as ser1:
    print(ser1.name)         # check which port was really used
    ser1.write(100)        # writes 100 bytes
with serial.Serial('COM4') as ser2:
    print(ser2.name)         # check which port was really used
    line = ser2.readline()   # reads line
    print(line)
#will not print for me
