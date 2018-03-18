Software - Python
We’ll go over in some detail here how to access and use the pi servo hat in Python.

Set Up Access to SMBus Resources

First point: in most OS level interactions, the I2C bus is referred to as SMBus. Thus we get our first lines of code. This imports the smbus module, creates an object of type SMBus, and attaches it to bus “1” of the Pi’s various SMBuses.

import smbus
bus = smbus.SMBus(1)

We have to tell the program the part’s address. By default, it is 0x40, so set a variable to that for later use.

addr = 0x40

Next, we want to enable the PWM chip and tell it to automatically increment addresses after a write (that lets us do single-operation multi-byte writes).

bus.write_byte_data(addr, 0, 0x20)
bus.write_byte_data(addr, 0xfe, 0x1e)

Write Values to the PWM Registers
That’s all the setup that needs to be done. From here on out, we can write data to the PWM chip and expect to have it respond. Here’s an example.

bus.write_word_data(addr, 0x06, 0)
bus.write_word_data(addr, 0x08, 1250)

The first write is to the “start time” register for channel 0. By default, the PWM frequency of the chip is 200Hz, or one pulse every 5ms. The start time register determines when the pulse goes high in the 5ms cycle. All channels are synchronized to that cycle. Generally, this should be written to 0. The second write is to the “stop time” register, and it controls when the pulse should go low. The range for this value is from 0 to 4095, and each count represents one slice of that 5ms period (5ms/4095), or about 1.2us. Thus, the value of 1250 written above represents about 1.5ms of high time per 5ms period.
Servo motors get their control signal from that pulse width. Generally speaking, a pulse width of 1.5ms yields a “neutral” position, halfway between the extremes of the motor’s range. 1.0ms yields approximately 90 degrees off center, and 2.0ms yields -90 degrees off center. In practice, those values may be slightly more or less than 90 degrees, and the motor may be capable of slightly more or less than 90 degrees of motion in either direction.
To address other channels, simply increase the address of the two registers above by 4. Thus, start time for channel 1 is 0x0A, for channel 2 is 0x0E, channel 3 is 0x12, etc. and stop time address for channel 1 is 0x0C, for channel 2 is 0x10, channel 3 is 0x14, etc. See the table below.

Channel #
Start Address
Stop Address

Ch 0
0x06
0x08

Ch 1
0x0A
0x0C

Ch 2
0x0E
0x10

Ch 3
0x12
0x14

Ch 4
0x16
0x18

Ch 5
0x1A
0x1C

Ch 6
0x1E
0x20

Ch 7
0x22
0x24

Ch 8
0x26
0x28

Ch 9
0x2A
0x2C

Ch 10
0x2E
0x30

Ch 11
0x32
0x34

Ch 12
0x36
0x38

Ch 13
0x3A
0x3C

Ch 14
0x3E
0x40

Ch 15
0x42
0x44

If you write a 0 to the start address, every degree of offset from 90 degrees requires 4.6 counts written to the stop address. In other words, multiply the number of degrees offset from neutral you wish to achieve by 4.6, then either add or subtract that result from 1250, depending on the direction of motion you wish. For example, a 45 degree offset from center would be 207 (45x4.6) counts either more or less than 1250, depending upon the direction you desire the motion to be in.
