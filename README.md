Platform Independent 6 DOF Complementary Filter
===============================================

This filter was created primarily for the MPU-6050 Gyroscope and Accelerometer module for the
Tiva C Launchpad by Texas Instruments written in TI's style (C written as object-oriented).
Because this is a generic complementary filter, it should be able to work with any 6 DOF
gyroscope and accelerometer module. I make no guarantees, but it should.
Since this library does not rely on any of TI's libraries, I have given it the MIT license.
Also, this library relies fairly heavily on floating point arithmetic and trigonometric
functions, so if used on a microcontroller without an FPU (floating point unit), this
library may be slow.

This C coded library can be used in any application requiring a 6 DOF Complementary Filter. No matter the platform (Arduino/AVR, PIC, MSP430, Kinetis, STM32, etc...), this library can be applied with absolutely no changing of the underlying implementation of the code. Include this library in any C/C++ project as needed.

All documentation for how to use the library can be found in the header file of the library.

Structures
==========================
tSixAxis


Functions
==========================

void SixCompInit(tSixAxis *psFilter, float fDeltaT, float fTau)
--------------------------------------------------------------------------------------------
psFilter: Address of a tSixAxis instantiation

fDeltaT:  The time delta between updates in seconds. fDeltaT = 1/(sampling frequency)

fTau:     Max allowable time in seconds until gyro drifts too far and comp. filter
          shifts its weight to the accelerometer. If the filter is not responding as fast
          as it should, tweak this value.
          
Description - Initializes the complementary filter.


void SixCompStart(tSixAxis *psFilter)
--------------------------------------------------------------------------------------------
psFilter: Address of a tSixAxis instantiation

Description - Should be called at least once before SixCompUpdate. This allows the comp.
filter to converge faster. If it is not called, the filter will still converge but it'll
take longer. Call SixCompAccelUpdate beforehand.


void SixCompUpdate(tSixAxis *psFilter)
--------------------------------------------------------------------------------------------
psFilter: Address of a tSixAxis instantiation

Description - Should be called on a regular interval defined by fDeltaT in order for the
filter to be accurate. SixCompAccelUpdate and SixCompGyroUpdate should be called prior to
calling SixCompUpdate.


void SixCompAnglesGet(tSixAxis *psFilter, float *pfXAngle, float *pfYAngle)
--------------------------------------------------------------------------------------------
psFilter: Address of a tSixAxis instantiation

pfXAngle: Address of float to store the X angle in radians into.

pfYAngle: Address of float to store the Y angle in radians into.

Description - Gets the angles (in the range of 0 to 2*pi) along the X and Y axes.


void SixCompAccelUpdate(tSixAxis *psFilter, float fAccelX, float fAccelY, float fAccelZ)
--------------------------------------------------------------------------------------------
psFilter: Address of a tSixAxis instantiation

fAccelX:  Accelerometer X value in m/s^2

fAccelY:  Accelerometer Y value in m/s^2

fAccelZ:  Accelerometer Z value in m/s^2

Description - Used for updating the Accelerometer values in the comp. filter.


void SixCompGyroUpdate(tSixAxis *psFilter, float fGyroX, float fGyroY, float fGyroZ)
--------------------------------------------------------------------------------------------
psFilter: Address of a tSixAxis instantiation

fGyroX:   Gyro X value in rad/sec

fGyroY:   Gyro Y value in rad/sec

fGyroZ:   Gyro Z value in rad/sec

Description - Used for updating the Gyroscope values in the comp. filter.
