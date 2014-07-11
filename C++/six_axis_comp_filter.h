//*********************************************************************************
// Six Axis Complementary Filter - Platform Independent
// 
// Revision: 1.2
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

// 
// Header Guard
// 
#ifndef SIX_AXIS_COMP_FILTER_H
#define SIX_AXIS_COMP_FILTER_H

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

// Converts a floating point number expressed in degrees to radians                  
#define RADIANS_F(deg)        ((deg)*0.0174532f)

// Converts a floating point number expressed in radians to degrees
#define DEGREES_F(rad)        ((rad)*57.2957795f)

//*********************************************************************************
// Class
//*********************************************************************************

class
SixAxisComp
{
    public:
        // 
        // Constructor
        // Description:
        //      Initializes the complementary filter.
        // Parameters:
        //      deltaT - The time delta between updates expressed in seconds.
        //      tau - Max allowable time until gyro drifts too far and comp. filter shifts
        //             its weight to the accelerometer expressed in seconds.     
        // Returns:
        //      None.
        // 
        SixAxisComp(float deltaT, float tau);
        
        // 
        // Complementary Filter Start
        // Description:
        //      Should be called once before CompUpdate can be called at the next interval.
        //      CompAccelUpdate must be called before this function.
        //      This function helps the filter to converge faster. If this function is not
        //      called, the filter will still converge, but it will take longer.
        // Parameters:
        //      None.
        // Returns:
        //      None.
        // 
        void CompStart();
        
        // 
        // Complementary Filter Update
        // Description:
        //      Must be called on a regular interval specified by deltaT.
        //      Be sure to call CompAccelUpdate and CompGyroUpdate with
        //      new values before calling this function.
        // Parameters:
        //      None.
        // Returns:
        //      None.
        // 
        void CompUpdate();
        
        // 
        // Complementary Filter Angles Get
        // Description:
        //      Acquires the angles in radians relative to ground along the positive
        //      X and Y axes.
        // Parameters:
        //      XAngle - Address of a float to store the angle relative to the X axis into.
        //               The number 0 can be passed as a parameter if this angle is not
        //               needed.
        //      YAngle - Address of a float to store the angle relative to the Y axis into.
        //               The number 0 can be passed as a parameter if this angle is not
        //               needed.
        // Returns:
        //      None.
        // 
        void CompAnglesGet(float *XAngle, float *YAngle);
        
        // 
        // Complementary Filter Accelerometer Update
        // Description:
        //      Updates the comp. filter with new accelerometer values.
        // Parameters:
        //      accelX - Acceleration vector along X axis expressed in m/s^2
        //      accelY - Acceleration vector along Y axis expressed in m/s^2
        //      accelZ - Acceleration vector along Z axis expressed in m/s^2
        // Returns:
        //      None.
        // 
        void CompAccelUpdate(float accelX, float accelY, float accelZ);
                                    
        // 
        // Complementary Filter Gyroscope Update
        // Description:
        //      Updates the comp. filter with new gyroscope values.
        // Parameters:
        //      gyroX - Angular velocity around X axis expressed in rad/s
        //      gyroY - Angular velocity around Y axis expressed in rad/s
        //      gyroZ - Angular velocity around Z axis expressed in rad/s
        // Returns:
        //      None.
        //                                
        void CompGyroUpdate(float gyroX, float gyroY, float gyroZ);
        
    private:
        //
        // The time delta between updates. deltaT = 1/(sampling frequency)
        //
        float deltaT;
        
        // 
        // Weighting factor
        // 
        float alpha;
        
        //
        // The most recent accelerometer readings.
        //
        float Ax, Ay, Az;
        
        //
        // The most recent gyroscope readings.
        //
        float Gx, Gy, Gz;
        
        // 
        // Accelerometer angles in relation to the X and Y axes.
        // 
        float accelAngleX, accelAngleY;
        
        // 
        // Comp. filter angle output in relation to the X and Y axes.
        // 
        float compAngleX, compAngleY;
        
        // 
        // Extrapolates angles according to accelerometer readings
        // 
        void CompAccelCalculate();
}

#endif  // SIX_AXIS_COMP_FILTER_H
