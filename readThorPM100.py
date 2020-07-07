from ThorlabsPM100 import ThorlabsPM100
import usbtmc
import time
import usb
from usb.core import find as finddev

dev = finddev(idVendor=0x1313, idProduct=0x8072)
dev.reset()

time.sleep(2)

print('Initializing PM100... ', end='')
instr =  usbtmc.Instrument(4883, 32882)
try:
 	print(instr.ask("*IDN?"))

except usb.core.USBError:
	print('...Failed! Exiting')
	sys.exit('ERROR: PM100 NOT DETECTED. TRY TURNING IT OFF AND ON AGAIN')

#rm = visa.ResourceManager()
#inst = rm.open_resource('USB0::0x0001::0x0007::13138072::INSTR', term_chars='\n', timeout=1) #Bus 001 Device 007: ID 1313:8072
#power_meter = ThorlabsPM100(inst=inst)
#inst = USBTMC(instr)

power_meter = ThorlabsPM100(inst=instr)
#power_meter.configure.scalar.power()  #pdensity()
power_meter.configure.scalar.pdensity()
power_meter.sense.average.count = 50
power_meter.system.beeper.immediate()
power_meter.sense.power.dc.range.auto = "ON"
power_meter.sense.correction.wavelength = 385
power_meter.input.pdiode.filter.lpass.state = 1

print( 'Measurement type: ', power_meter.getconfigure)
if power_meter.getconfigure == "PDEN":
	print( ' Beam diameter is ' + str(power_meter.sense.correction.beamdiameter) + 'mm' )
	print( ' Units are ' + str(power_meter.sense.power.dc.unit) + '/cm^2' )
else:
	print( ' Units are ' + str(power_meter.sense.power.dc.unit) )

print( 'Wavelength: ' + str(power_meter.sense.correction.wavelength) + ', ' + str(power_meter.sense.correction.power.pdiode.response) + ' A/W' )
print( 'Averaging: ' + str(power_meter.sense.average.count) ) # read property
print( 'Power range limit: ', power_meter.sense.power.dc.range.upper)
print( '' )

input("Cover sensor and press enter to meausure zero")

print('Measuering zero power level...')

power_meter.sense.correction.collect.zero.initiate()
while power_meter.sense.correction.collect.zero.state:
	time.sleep(0.5)

print( '' )
input("Zero power measured and applied. Uncover sensor and press enter to continue")

print( '' )
print('Measuering power level...')
try:
	while True:
		print( power_meter.read ) # Read-only property
		time.sleep(0.5)
except KeyboardInterrupt:
	pass

print('Caught interrupt, closing PM100')

instr.close()
del instr
