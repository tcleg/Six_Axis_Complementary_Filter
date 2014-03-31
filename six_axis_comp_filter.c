//*********************************************************************************
// Six Axis Complementary Filter
// 
// Revision: 1
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

//*********************************************************************************
// Headers
//*********************************************************************************
#include <math.h>
#include "six_axis_comp_filter.h"

//*********************************************************************************
// Functions
//*********************************************************************************
void
SixCompAccelCalculate(tSixAxis *psFilter)
{
    // Make data easier to visualize
    float Ax = psFilter->pfAccel[0];
    float Ay = psFilter->pfAccel[1];
    float Az = psFilter->pfAccel[2];
    float ang[2];
    uint8_t idx;
    
    // Angle made by X axis acceleration vector relative to ground
    ang[0] = atan2f(Ax, sqrtf( SQ(Ay) + SQ(Az) ) );
    
    // Angle made by Y axis acceleration vector relative to ground
    ang[1] = atan2f(Ay, sqrtf( SQ(Ax) + SQ(Az) ) );
    
    // Check to see which quadrant of the unit circle the angle lies in
    // and format the angle to lie in the range of 0 to 2*PI
    for(idx = 0; idx < 2; idx++)
    {
        
        /*
        // Commented out because it does nothing
        if(Az > 0.0 && ang[idx] >= 0.0)
        {
            // Angle lies in Quadrant 1
            // Do nothing
        }
        */
        
        if(Az < 0.0f && ang[idx] >= 0.0f)
        {
            // Angle lies in Quadrant 2
            ang[idx] = PI - ang[idx];
        }
        else if(Az < 0.0f && ang[idx] <= 0.0f)
        {
            // Angle lies in Quadrant 3
            ang[idx] = PI - ang[idx];
        }
        else if(Az > 0.0f && ang[idx] < 0.0f)
        {
            // Angle lies in Quadrant 4
            ang[idx] = TWO_PI + ang[idx];
        }
        
        // Save accelerometer angle to structure
        psFilter->pfAccelAngle[idx] = ang[idx];
    }
}

void 
SixCompInit(tSixAxis *psFilter, float fDeltaT, float fTau)
{
    // Save values to structure
    psFilter->fDeltaT = fDeltaT;
    psFilter->fTau = fTau;
    
    // Calculate weighting factor
    psFilter->fAlpha = fTau/(fTau + fDeltaT);
}


void 
SixCompStart(tSixAxis *psFilter)
{
    // Calculate accelerometer angles
    SixCompAccelCalculate(psFilter);
    
    // Initialize filter to accel angles
    psFilter->pfCompAngle[0] = psFilter->pfAccelAngle[0];
    psFilter->pfCompAngle[1] = psFilter->pfAccelAngle[1];
}


void 
SixCompUpdate(tSixAxis *psFilter)
{   
    // Make the data easier to work with and visualize.
    float alpha = psFilter->fAlpha;
    float deltaT = psFilter->fDeltaT;
    float omega;
    float accAng[2];
    float comp[2];
    uint8_t idx;
    
    // Calculate accelerometer angles
    SixCompAccelCalculate(psFilter);
    accAng[0] = psFilter->pfAccelAngle[0];
    accAng[1] = psFilter->pfAccelAngle[1];
    
    for(idx = 0; idx < 2; idx++)
    {
        comp[idx] = psFilter->pfCompAngle[idx];
        
        // Work with angles that are closest in distance to the accelerometer angle
        // Angle > Accelerometer Angle
        if(comp[idx] > accAng[idx])
        {
            // accelAngle + (2*pi - Angle) < Angle - accelAngle
            if( accAng[idx] + (TWO_PI - comp[idx]) < comp[idx] - accAng[idx] )
            {
                // Angle = Angle - 2*pi
                comp[idx] = comp[idx] - TWO_PI;
            }
        } 
        else
        {
            // Angle + (2*pi - accelAngle) < accelAngle - Angle
            if( comp[idx] + (TWO_PI - accAng[idx]) < accAng[idx] - comp[idx] )
            {
                // Angle = Angle + 2*pi
                comp[idx] = comp[idx] + TWO_PI;
            }
        }
        
        // Complementary Filter for angle X
        if( idx == 0)
        {
            // Take rotational velocity about the Y axis and invert the sense of
            // direction
            omega = -(psFilter->pfGyro[1]);
        
            // AngleX = alpha*(AngleX + omegaY*deltaT) + (1 - alpha)*accelAngleX
            comp[idx] = alpha*(comp[idx] + omega*deltaT) + (1.0f - alpha)*accAng[idx];
        }
        // Complementary Filter for angle Y
        else
        {
            // Take rotational velocity about the X axis
            omega = psFilter->pfGyro[0];
        
            // AngleY = alpha*(AngleY + omegaX*deltaT) + (1 - alpha)*accelAngleY
            comp[idx] = alpha*(comp[idx] + omega*deltaT) + (1.0f - alpha)*accAng[idx];
        }
        
        // Format comp. outputs to always be within the range of 0 to 2*PI
        while(comp[idx] >= TWO_PI)
        {
            comp[idx] = comp[idx] - TWO_PI;
        }
        
        while(comp[idx] < 0.0f)
        {
            comp[idx] = comp[idx] + TWO_PI;
        }
        
        // Save comp. filter value
        psFilter->pfCompAngle[idx] = comp[idx];
    }
}


void 
SixCompAnglesGet(tSixAxis *psFilter, float *pfCompAngleX, float *pfCompAngleY)
{
    // Transfer structure's updated comp. filter's angles
    *pfCompAngleX = psFilter->pfCompAngle[0];
    *pfCompAngleY = psFilter->pfCompAngle[1];
}


void 
SixCompAccelUpdate(tSixAxis *psFilter, float fAccelX, float fAccelY, float fAccelZ)
{
    // Save values to structure
    psFilter->pfAccel[0] = fAccelX;
    psFilter->pfAccel[1] = fAccelY;
    psFilter->pfAccel[2] = fAccelZ;
}


void 
SixCompGyroUpdate(tSixAxis *psFilter, float fGyroX, float fGyroY, float fGyroZ)
{
    // Save values to structure
    psFilter->pfGyro[0] = fGyroX;
    psFilter->pfGyro[1] = fGyroY;
    psFilter->pfGyro[2] = fGyroZ;
}


