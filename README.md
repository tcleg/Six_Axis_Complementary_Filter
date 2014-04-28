6 DOF Complementary Filter
===============================================

Platform Independent

This filter was created primarily for the MPU-6050 Gyroscope and Accelerometer module for the
Tiva C Launchpad by Texas Instruments written in TI's style (C written as object-oriented).
Because this is a generic complementary filter, it should be able to work with any 6 DOF
gyroscope and accelerometer module. I make no guarantees, but it should.
Since this library does not rely on any of TI's libraries, I have given it the MIT license.
Also, this library relies fairly heavily on floating point arithmetic and trigonometric
functions, so if used on a microcontroller without an FPU (floating point unit), this
library may be slow.

This C coded library can be used in any application requiring a 6 DOF Complementary Filter. 
No matter the platform (Arduino/AVR, PIC, MSP430, Kinetis, STM32, etc...), this library can 
be applied with absolutely no changing of the underlying implementation of the code. Include 
this library in any C/C++ project as needed.

All documentation for how to use the library can be found in the header file of the library.
