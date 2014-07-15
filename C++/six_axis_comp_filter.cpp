//*********************************************************************************
// Six Axis Complementary Filter - Platform Independent
// 
// Revision: 1.5
// 
// Description: Takes gyroscope and accelerometer readings and produces a "fused"
// reading that is more accurate. Relies heavily on floating point arithmetic
// and trigonometry.
// 
// This library has been tested to work with the MPU-6050 6 DOF IMU Sensor
// comprising a gyroscope and accelerometer. It should also work with other 6 
// DOF IMUs but there are no guarantees. Also, this library should work 
// with even 5 DOF IMUs since the gyroscope reading about the Z axis technically
// goes unused with this library at its current revision.
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

//*********************************************************************************
// Headers
//*********************************************************************************
#include <math.h>
#include "six_axis_comp_filter.h"

//*********************************************************************************
// Macros and Globals
//*********************************************************************************
#define PI                              3.1415926f
#define HALF_PI                         1.5707963f
#define TWO_PI                          6.2831853f
#define SQRE(x) 		                ((x)*(x))

//*********************************************************************************
// Public Class Functions
//*********************************************************************************

CompSixAxis::
CompSixAxis(float deltaTime, float tau)
{
    // Save value to class
    deltaT = deltaTime;
    
    // Calculate weighting factor
    alpha = tau/(tau + deltaT);
    
    // Initialize other class variables
    compAngleX = 0;
    compAngleY = 0;
    accelAngleX = 0;
    accelAngleY = 0;
    Ax = 0;
    Ay = 0;
    Az = 0;
    Gx = 0;
    Gy = 0;
    Gz = 0;
}

void CompSixAxis::
CompStart()
{
    // Calculate accelerometer angles
    CompAccelCalculate();
    
    // Initialize filter to accel angles
    compAngleX = accelAngleX;
    compAngleY = accelAngleY;
}

void CompSixAxis::
CompUpdate()
{   
    // Calculate the accelerometer angles
    CompAccelCalculate();
    
    // Omega is the rotational velocity reported by the gyroscope. Though it seems
    // strange, the rotational velocity about the Y axis must be projected back
    // onto the X axis and then its sense of direction must be inverted in order to
    // acquire positive angles about the X axis. This is shown below with -Gy being
    // passed as a parameter.
    compAngleX = CompFilterProcess(compAngleX, accelAngleX, -Gy);
    
    // In this case, the rotational velocity about the X axis (Gx) is projected back
    // onto the Y axis and its sense of direction is already correct.
    compAngleY = CompFilterProcess(compAngleY, accelAngleY, Gx);
}

void CompSixAxis::
CompAnglesGet(float *XAngle, float *YAngle)
{
    // Transfer class's updated comp. filter's angles
    // Check if valid addresses were passed as well.
    if(XAngle)
    {
        *XAngle = compAngleX;
    }
    if(YAngle)
    {
        *YAngle = compAngleY;
    }
}

void CompSixAxis::
CompAccelUpdate(float accelX, float accelY, float accelZ)
{
    // Save values to class
    Ax = accelX;
    Ay = accelY;
    Az = accelZ;
}

void CompSixAxis::
CompGyroUpdate(float gyroX, float gyroY, float gyroZ)
{
    // Save values to class
    Gx = gyroX;
    Gy = gyroY;
    Gz = gyroZ;
}

//*********************************************************************************
// Private Class Functions
//*********************************************************************************

// 
// Calculates the angles according to the accelerometer based on the acceleration
// readings
// 
void CompSixAxis::
CompAccelCalculate()
{
    // Angle made by X axis acceleration vector relative to ground
    angleAngleX = atan2f(Ax, sqrtf( SQRE(Ay) + SQRE(Az) ) );
    
    // Angle made by Y axis acceleration vector relative to ground
    angleAngleY = atan2f(Ay, sqrtf( SQRE(Ax) + SQRE(Az) ) );
    
    // Format the accel. angles to lie in the range of 0 to 2*pi
    accelAngleX = FormatAccelRange(accelAngleX, Az);
    accelAngleY = FormatAccelRange(accelAngleY, Az);
}

// 
// Check to see which quadrant of the unit circle the angle lies in
// and format the angle to lie in the range of 0 to 2*PI
// 
float CompSixAxis::
FormatAccelRange(float accelAngle, float accelZ)
{
    if(accelZ < 0.0f)
    {
        // Angle lies in Quadrant 2 or Quadrant 3 of
        // the unit circle
        accelAngle = PI - accelAngle;
    }
    else if(accelZ > 0.0f && accelAngle < 0.0f)
    {
        // Angle lies in Quadrant 4 of the unit circle
        accelAngle = TWO_PI + accelAngle;
    }
    
    // If both of the previous conditions were not satisfied, then
    // the angle must lie in Quadrant 1 and nothing more needs
    // to be done.
    
    return accelAngle;
}

// 
// Formats the complimentary filter angle for faster convergence of the filter.
// 
float CompSixAxis::
FormatFastConverge(float compAngle, float accAngle)
{
    // Work with comp. angles that are closest in distance to the accelerometer angle
    // on the unit circle. This allows for significantly faster filter convergence.
    if(compAngle > accAngle + PI)
    {
        compAngle = compAngle - TWO_PI;
    }
    else if(accAngle > compAngle + PI)
    {
        compAngle = compAngle + TWO_PI;
    }
    
    return compAngle;
}

// 
// Formats the complimentary filter angle to always lie within the range of
// 0 to 2*pi
// 
float CompSixAxis::
FormatRange0to2PI(float compAngle)
{
    while(compAngle >= TWO_PI)
    {
        compAngle = compAngle - TWO_PI;
    }
    
    while(compAngle < 0.0f)
    {
        compAngle = compAngle + TWO_PI;
    }
    
    return compAngle;
}

// 
// Complimentary Filter - This is where the magic happens.
// 
float CompSixAxis::
CompFilterProcess(float compAngle, float accelAngle, float omega)
{
    float gyroAngle;

    // Speed up filter convergence
    compAngle = FormatFastConverge(compAngle, accelAngle);
    
    // Integrate the gyroscope's angular velocity reading to get an angle
    gyroAngle = compAngle + omega*deltaT;
    
    // Weighting is applied to the gyroscope's angular position and 
    // accelerometer's angular position and they are put together to form one 
    // angle, the complementary filter angle.
    compAngle = alpha*gyroAngle + (1.0f - alpha)*accelAngle;
    
    // Format the Comp. Angle to lie in the range of 0 to 2*pi
    compAngle = FormatRange0to2PI(compAngle);
    
    return compAngle;
}
