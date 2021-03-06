////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMULib.h"
#include "pid.h"
#include <wiringPi.h>
#include <unistd.h>
#include <wiringSerial.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <fstream>

using namespace std;

//motor speed: represented as delay time in microseconds
double motor_delay[4];

const double dt = 0.01;
const double pitch_offset = 0.835;
const double roll_offset = 1.425;
const double delay_lower_limit = 500;
const double delay_upper_limit = 9999999;
const double speed_lower_limit = 0.000201;
const double speed_upper_limit = 4;

//imu data
double imu_roll;
double imu_pitch;
double imu_yaw;

//pids
PID *pid_roll;
PID *pid_pitch;

double acc_roll;
double acc_pitch;

double pid_pitch_speed;//
double pid_roll_speed;//

double pid_pitch_delay;
double pid_roll_delay;

void computeMotorSpeed(double roll, double pitch, double yaw)
{
    if(fabs(pitch) > 0.000001)
    {
        acc_pitch = pid_pitch->calculate(pitch_offset, pitch);
        pid_pitch_speed += acc_pitch * dt;
        if (fabs(pid_pitch_speed) > speed_upper_limit)
        {
            pid_pitch_speed = pid_pitch_speed > 0 ? speed_upper_limit : -speed_upper_limit;
        }
        if(fabs(pid_pitch_speed) < speed_lower_limit)
        {
            pid_pitch_delay = pid_pitch_speed > 0 ? delay_upper_limit : -delay_upper_limit;
        }
        else
        {
            pid_pitch_delay = 2000 / pid_pitch_speed;
        }
        if (fabs(pid_pitch_delay) < delay_lower_limit)
        {
            pid_pitch_delay = pid_pitch_delay > 0 ? delay_lower_limit : -delay_lower_limit;
        }
        motor_delay[0] = -pid_pitch_delay;
        motor_delay[2] = pid_pitch_delay;

    }
    
    if(fabs(roll) > 0.000001)
    {
        acc_roll = pid_roll->calculate(roll_offset, roll);
        pid_roll_speed += acc_roll * dt;
         if (fabs(pid_roll_speed) > speed_upper_limit)
        {
            pid_roll_speed = pid_roll_speed > 0 ? speed_upper_limit : -speed_upper_limit;
        }
        if(fabs(pid_roll_speed) < speed_lower_limit)
        {
            pid_roll_delay = pid_roll_speed > 0 ? delay_upper_limit : -delay_upper_limit;
        }
        else
        {
            pid_roll_delay = 2000 / pid_roll_speed;
        }
        if (fabs(pid_roll_delay) < delay_lower_limit)
        {
            pid_roll_delay = pid_roll_delay > 0 ? delay_lower_limit : -delay_lower_limit;
        }
        motor_delay[1] = -pid_roll_delay;
        motor_delay[3] = pid_roll_delay;
        
    }
}

int main()
{
    int sampleCount = 0;
    int sampleRate = 0;
    uint64_t rateTimer;
    uint64_t displayTimer;
    uint64_t now;

    //for arduino communication
    int fd ;
    char str[80];
    
    if ((fd = serialOpen ("/dev/ttyACM0", 115200)) < 0)
    {
      fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
      return 1 ;
    }

    if (wiringPiSetup () == -1)
    {
      fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
      return 1 ;
    }
    
    //  Using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.
    //  Or, you can create the .ini in some other directory by using:
    //      RTIMUSettings *settings = new RTIMUSettings("<directory path>", "RTIMULib");
    //  where <directory path> is the path to where the .ini file is to be loaded/saved

    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  This is an opportunity to manually override any settings before the call IMUInit

    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    //  set up for rate timer

    rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();

    //  now just process data
    ifstream fin;
    fin.open("pid.txt");
    double p,i,d;
    fin >> p >> i >> d;
    printf("PID param: P: %f I: %f D: %f\r", p, i, d);
    fin.close();
    pid_roll = new PID(0.01, 1000000, -1000000, p, d, i);
    pid_pitch = new PID(0.01, 1000000, -1000000, p, d, i);
    pid_pitch_speed = 0;
    pid_roll_speed = 0;

    while (1) {
        //  poll at the rate recommended by the IMU

        usleep(imu->IMUGetPollInterval() * 1000);
        RTIMU_DATA imuData;
        while (imu->IMURead()) {
            imuData= imu->getIMUData();
            sampleCount++;

            now = RTMath::currentUSecsSinceEpoch();

            //  display 10 times per second

            if ((now - displayTimer) > 100000) {
                imu_roll = imuData.fusionPose.x() * RTMATH_RAD_TO_DEGREE;
                imu_pitch = imuData.fusionPose.y() * RTMATH_RAD_TO_DEGREE;
                imu_yaw = imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
                computeMotorSpeed(imu_roll, imu_pitch, imu_yaw);
                //sprintf(str, "<%#.1f:%#.1f:%#.1f:%#.1f>", motor_delay[0], motor_delay[1], motor_delay[2], motor_delay[3]);
                sprintf(str, "<%d:%d:%d:%d>", (int)motor_delay[0], (int)motor_delay[1], (int)motor_delay[2], (int)motor_delay[3]);
                serialPuts(fd, str);
                serialFlush (fd);
                sleep(100000);
                printf("delay: m1: %f m2: %f m3: %f m4: %f\r", motor_delay[0], motor_delay[1], motor_delay[2], motor_delay[3]);
                //printf("imu data: roll: %f pitch: %f yaw: %f\r", imu_roll, imu_pitch, imu_yaw);
                //printf("Sample rate %d: %s\r", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));
                fflush(stdout);
                displayTimer = now;
            }

            //  update rate every second

            if ((now - rateTimer) > 1000000) {
                sampleRate = sampleCount;
                sampleCount = 0;
                rateTimer = now;
            }
        }
    }
}

