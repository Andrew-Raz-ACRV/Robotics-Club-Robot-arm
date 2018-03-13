#!usr/bin/env python

import smbus, time

bus = smbus.SMBus(1)
addr = 0x40

bus.write_byte_data(addr, 0, 0x20)
bus.write_byte_data(addr, 0xfe, 0x1e)

st0 = 0x06
sp0 = 0x08

#Initialise Motors at start
bus.write_word_data(addr, st0, 0) #ch0
bus.write_word_data(addr, st0+1, 0) #ch3
bus.write_word_data(addr, 0x16, 0) #ch4
bus.write_word_data(addr, 0x22, 0) #ch7
bus.write_word_data(addr, 0x26, 0) #ch8

#Motor control
bus.write_word_data(addr, 0x08, 0)

print('Start Ch0 motor')

bus.write_word_data(addr, 0x08, 1250)  #1250

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x08, 836) #836

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x08, 1664) #1664

time.sleep(2)

print('Finished Ch0 motor')

bus.write_word_data(addr, 0x14, 0)

print('Start Ch3 motor')

bus.write_word_data(addr, 0x14, 1250)  #1250

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x14, 836) #836

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x14, 1664) #1664

time.sleep(2)

print('Finished Ch3 motor')

bus.write_word_data(addr, 0x18, 0)

print('Start Ch4 motor')

bus.write_word_data(addr, 0x18, 1250)

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x18, 836)

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x18, 1664)

time.sleep(2)

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x18, 836)

print('Finished Ch4 motor')

time.sleep(2)

bus.write_word_data(addr, 0x24, 0)

print('Start Ch7 motor')

bus.write_word_data(addr, 0x24, 1250)  #1250

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x24, 836) #836

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x24, 1250)  #1250

time.sleep(2)

bus.write_word_data(addr, 0x24, 836) #836

bus.write_word_data(addr, 0x24, 1250)  #1250

time.sleep(2)

print('Finished Ch7 motor')

bus.write_word_data(addr, 0x28, 0)

print('Start Ch8 motor')

bus.write_word_data(addr, 0x28, 1250)  #1250

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x28, 836) #836

print('Start')

time.sleep(2)

bus.write_word_data(addr, 0x28, 1664) #1664

time.sleep(2)

print('Finished Ch8 motor')

#Turn off all the servos
bus.write_word_data(addr, 0x08, 0)
bus.write_word_data(addr, 0x14, 0)
bus.write_word_data(addr, 0x18, 0)
bus.write_word_data(addr, 0x24, 0)
bus.write_word_data(addr, 0x28, 0)
