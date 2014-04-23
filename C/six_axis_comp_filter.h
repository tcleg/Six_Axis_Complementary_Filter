//*********************************************************************************
// Six Axis Complementary Filter - Platform Independent
// 
// Revision: 1.0
// 
// Description: Takes gyroscope and accelerometer readings and produces a "fused"
// reading that is more accurate. Relies heavily on floating point arithmetic
// and trigonometry.
// 
// This library has been tested to work with the MPU-6050 6 DOF IMU Sensor
// comprising a gyroscope and accelerometer. It should also work with other 6 
// DOF IMUs but there are no guarantees. Also, this library should work 
// with even 5 DOF IMUs since the gyroscope reading about the Z axis technically
// goes unused with this library at its current Revision 1.0.
// 
// Revisions can be found here:
// https://github.com/tcleg
// 
// Copyright (C) 2014 Trent Cleghorn , <trentoncleghorn@gmail.com>
// 
//                                  MIT License
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//*********************************************************************************

// 
// Header Guard
// 
#ifndef SIX_AXIS_COMP_FILTER_H
#define SIX_AXIS_COMP_FILTER_H

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

// Converts a floating point number expressed in degrees to radians                  
#define RADIANS_F(deg)        ((deg)*0.0174532f)

// Converts a floating point number expressed in radians to degrees
#define DEGREES_F(rad)        ((rad)*57.2957795f)

typedef struct
{
    //
    // The time delta between updates. fDeltaT = 1/(sampling frequency)
    //
    float fDeltaT;

    // 
    // Max allowable time until gyro drifts too far and comp. filter shifts
    // its weight to the accelerometer.
    // 
    float fTau;
    
    // 
    // Weighting factor
    // 
    float fAlpha;
    
    //
    // The most recent accelerometer readings.
    //
    float pfAccel[3];

    //
    // The most recent gyroscope readings.
    //
    float pfGyro[3];
    
    // 
    // Angle in relation to the X and Y axes.
    // 
    float pfAccelAngle[2];
    
    // 
    // Comp. Filter Angle Output of X and Y axes.
    // 
    float pfCompAngle[2];
}
tSixAxis;


//*********************************************************************************
// Prototypes
//*********************************************************************************

// 
// Six Axis Complementary Filter Initialize
// Description:
//      Initializes the complementary filter.
// Parameters:
//      psFilter - A tSixAxis instantiation.
//      fDeltaT - The time delta between updates expressed in seconds.
//      fTau - Max allowable time until gyro drifts too far and comp. filter shifts
//             its weight to the accelerometer expressed in seconds.     
// Returns:
//      Nothing.
// 
extern void SixCompInit(tSixAxis *psFilter, float fDeltaT, float fTau);

// 
// Six Axis Complementary Filter Start
// Description:
//      Should be called once before SixCompUpdate can be called at the next interval.
//      SixCompAccelUpdate must be called before this function.
//      This function helps the filter to converge faster. If this function is not
//      called, the filter will still converge, but it will take longer.
// Parameters:
//      psFilter - A tSixAxis instantiation.
// Returns:
//      Nothing.
// 
extern void SixCompStart(tSixAxis *psFilter);

// 
// Six Axis Complementary Filter Update
// Description:
//      Must be called on a regular interval specified by fDeltaT.
// Parameters:
//      psFilter - A tSixAxis instantiation.
// Returns:
//      Nothing.
// 
extern void SixCompUpdate(tSixAxis *psFilter);

// 
// Six Axis Complementary Filter Angles Get
// Description:
//      Acquires the angles in radians relative to ground along the positive
//      X and Y axes.
// Parameters:
//      psFilter - A tSixAxis instantiation.
//      pfXAngle - Address of a float to store the angle relative to the X axis into.
//      pfYAngle - Address of a float to store the angle relative to the Y axis into.
// Returns:
//      Nothing.
// 
extern void SixCompAnglesGet(tSixAxis *psFilter, float *pfXAngle,
                             float *pfYAngle);

// 
// Six Axis Complementary Filter Accelerometer Update
// Description:
//      Updates the comp. filter with new accelerometer values.
// Parameters:
//      psFilter - A tSixAxis instantiation.
//      fAccelX - Acceleration vector along X axis expressed in m/s^2
//      fAccelY - Acceleration vector along Y axis expressed in m/s^2
//      fAccelZ - Acceleration vector along Z axis expressed in m/s^2
// Returns:
//      Nothing.
// 
extern void SixCompAccelUpdate(tSixAxis *psFilter, float fAccelX, float fAccelY,
                               float fAccelZ);
                               
// 
// Six Axis Complementary Filter Gyroscope Update
// Description:
//      Updates the comp. filter with new gyroscope values.
// Parameters:
//      psFilter - A tSixAxis instantiation.
//      fGyroX - Angular velocity around X axis expressed in rad/s
//      fGyroY - Angular velocity around Y axis expressed in rad/s
//      fGyroZ - Angular velocity around Z axis expressed in rad/s
// Returns:
//      Nothing.
//                                
extern void SixCompGyroUpdate(tSixAxis *psFilter, float fGyroX, float fGyroY,
                              float fGyroZ);

// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif  // SIX_AXIS_COMP_FILTER_H
