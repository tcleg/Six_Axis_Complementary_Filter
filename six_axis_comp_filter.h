//*********************************************************************************
// Six Axis Complementary Filter
// 
// Revision: 0
// 
// Description: Takes gyroscope and accelerometer readings and produces a "fused"
// reading that is more accurate. Relies heavily on floating point arithmetic
// and trigonometry.
// 
// Copyright (C) 2014 Trent Cleghorn
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, andór sell
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

#define PI                              3.1415926535897932384626433832795
#define HALF_PI                         1.5707963267948966192313216916398
#define TWO_PI                          6.283185307179586476925286766559
#define DEG_TO_RAD                      0.017453292519943295769236907684886
#define RAD_TO_DEG                      57.295779513082320876798154814105
#define RADIANS(deg)                    ((deg)*DEG_TO_RAD)
#define DEGREES(rad)                    ((rad)*RAD_TO_DEG)
#define SQ(x) 		                    ((x)*(x))


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

extern void SixCompInit(tSixAxis *psFilter, float fDeltaT, float fTau);

// 
// SixCompStart must be called once before SixCompUpdate can be called at the next interval.
// SixCompAccelUpdate must be called beforehand
// 
extern void SixCompStart(tSixAxis *psFilter);

// 
// SixCompUpdate must be called at a regular interval specified by fDeltaT
// 
extern void SixCompUpdate(tSixAxis *psFilter);

// 
// SixCompAnglesGet acquires the angles in radians relative to ground
// along the X and Y axes.
// 
extern void SixCompAnglesGet(tSixAxis *psFilter, float *pfXAngle,
                                 float *pfYAngle);

//                                  
// SixCompAccelUpdate and SixCompGyroUpdate must be called before calling SixCompUpdate
// 
extern void SixCompAccelUpdate(tSixAxis *psFilter, float fAccelX, float fAccelY,
                               float fAccelZ);
extern void SixCompGyroUpdate(tSixAxis *psFilter, float fGyroX, float fGyroY,
                              float fGyroZ);

// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif  // SIX_AXIS_COMP_FILTER_H
