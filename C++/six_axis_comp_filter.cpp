//*********************************************************************************
// Six Axis Complementary Filter - Platform Independent
// 
// Revision: 1.4
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
CompSixAxis(float deltaT, float tau)
{
    // Save value to structure
    deltaT = deltaT;
    
    // Calculate weighting factor
    alpha = tau/(tau + deltaT);
}

void CompSixAxis::
CompStart()
{
    // Calculate accelerometer angles
    SixCompAccelCalculate();
    
    // Initialize filter to accel angles
    compAngleX = accelAngleX;
    compAngleY = accelAngleY;
}

void CompSixAxis::
CompUpdate()
{   
    // Make the data easier to work with and visualize.
    uint8_t idx;
    float omega;
    float accAng[2];
    float comp[2];
    
    // Calculate accelerometer angles
    SixCompAccelCalculate();
    
    // Gather the current accelerometer and complementary filter angles
    accAng[0] = accelAngleX;
    accAng[1] = accelAngleY;
    comp[0] = compAngleX;
    comp[1] = compAngleY;
    
    for(idx = 0; idx < 2; idx++)
    {
        // Work with comp. angles that are closest in distance to the accelerometer angle
        // on the unit circle. This allows for significantly faster filter convergence.
        if(comp[idx] > accAng[idx] + PI)
        {
            comp[idx] = comp[idx] - TWO_PI;
        }
        else if(accAng[idx] > comp[idx] + PI)
        {
            comp[idx] = comp[idx] + TWO_PI;
        }
        
        // Acquire the correct gyroscopic angular velocity
        if( idx == 0)
        {
            // Take rotational velocity about the Y axis and invert the sense of
            // direction
            omega = -Gy;
        }
        else
        {
            // Take rotational velocity about the X axis
            omega = Gx;
        }
        
        // Complementary Filter - This is where the magic happens.
        comp[idx] = alpha*(comp[idx] + omega*deltaT) + (1.0f - alpha)*accAng[idx];
        
        // Format comp. outputs to always be within the range of 0 to 2*pi
        while(comp[idx] >= TWO_PI)
        {
            comp[idx] = comp[idx] - TWO_PI;
        }
        
        while(comp[idx] < 0.0f)
        {
            comp[idx] = comp[idx] + TWO_PI;
        }
        
        // Save comp. filter value
        if(idx == 0)
        {
            compAngleX = comp[idx];
        }
        else
        {
            compAngleY = comp[idx];
        }
        
    }
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

void CompSixAxis::
CompAccelCalculate()
{
    uint8_t idx;
    float angle[2];
    
    // Angle made by X axis acceleration vector relative to ground
    // accelAngleX = atan(Ax, sqrt( SQ(Ay) + SQ(Az) )
    angle[0] = atan2f(Ax, sqrtf( SQRE(Ay) + SQRE(Az) ) );
    
    // Angle made by Y axis acceleration vector relative to ground
    // accelAngleY = atan(Ay, sqrt( SQ(Ax) + SQ(Az) )
    angle[1] = atan2f(Ay, sqrtf( SQRE(Ax) + SQRE(Az) ) );
    
    // Check to see which quadrant of the unit circle the angle lies in
    // and format the angle to lie in the range of 0 to 2*PI
    for(idx = 0; idx < 2; idx++)
    {
        if(Az < 0.0f)
        {
            // Angle lies in Quadrant 2 or Quadrant 3 of
            // the unit circle
            angle[idx] = PI - angle[idx];
        }
        else if(Az > 0.0f && angle[idx] < 0.0f)
        {
            // Angle lies in Quadrant 4 of the unit circle
            angle[idx] = TWO_PI + angle[idx];
        }
        
        // If both of the previous conditions were not satisfied, then
        // the angle must lie in Quadrant 1
        
        // Save accelerometer angle to class
        if(idx == 0)
        {
            accelAngleX = angle[idx];
        }
        else
        {
            accelAngleY = angle[idx];
        }
    }
}

