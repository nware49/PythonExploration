#!/usr/bin/python3
#
# based on example at
# https://github.com/pimoroni/ads1015-python/blob/master/examples/read-all.py
#

from ads1015 import ADS1015

# calibrations of resistive dividers measured with voltmeter
a0cal = 4.108
a1cal = 4.123
a2cal = 4.095
a3cal = 4.126

ads = ADS1015()

ads.set_mode('single')
ads.set_programmable_gain(4.096)

a0 = ads.get_voltage('in0/gnd')
a1 = ads.get_voltage('in1/gnd')
a2 = ads.get_voltage('in2/gnd')
a3 = ads.get_voltage('in3/gnd')

print('Battery 1: %.3fV (%.3f)' % ( (a0*a0cal), a0 ) )
print('Battery 2: %.3fV (%.3f)' % ( (a1*a1cal), a1 ) )
print('  12 volt: %.3fV (%.3f)' % ( (a2*a2cal), a2 ) )
print('   5 volt: %.3fV (%.3f)' % ( (a3*a3cal), a3 ) )

