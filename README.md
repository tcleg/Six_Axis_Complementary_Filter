6 DOF Complementary Filter
===============================================

Platform Independent

This filter was created primarily for the MPU-6050 Gyroscope and Accelerometer module for the
Tiva C Launchpad by Texas Instruments written in TI's style (C written as object-oriented).
Because this is a generic complementary filter, it should be able to work with any 6 DOF
gyroscope and accelerometer module. I make no guarantees, but it should. The only problem
that might be encountered is in relation to the orientation of axes of the sensor(s). If this
problem occurs, consult the MPU6050 datasheet and examine how its orientation is laid out.
The IMU sensor being used must have the same orientation for both the gyroscope and
accelerometer.

Since this library does not rely on any of TI's libraries, I have given it the MIT license.
Also, this library relies fairly heavily on floating point arithmetic and trigonometric
functions, so if used on a microcontroller without an FPU (floating point unit), this
library may be slow.

This C coded library can be used in any application requiring a 6 DOF Complementary Filter. 
No matter the platform (Arduino/AVR, PIC, MSP430, Kinetis, STM32, etc...), this library can 
be applied with absolutely no changing of the underlying implementation of the code. Include 
this library in any C/C++ project as needed.

All documentation for how to use the library can be found in the header file of the library.

An example implementation with Arduino and the LSM6DS3 motion sensor can be found in (this gist)[https://gist.github.com/savovs/cbe998c3dfea711c3413cb23b6244cd9].
