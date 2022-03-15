#include "drive.h"
#include "Arduino.h"
#include "DRV8833_MCPWM.h"

//linear interpolation of speed from test data
int vel2pwm(float speed)
{
    int pwm = 0;
    if (speed<0)
    {
        pwm = (speed-4)/0.2666666667;
    }
    else if (speed>0)
    {
        pwm = (speed+4)/0.2666666667;
    }
    if (pwm > 100)
    {
        pwm = 100;
    }
    else if (pwm<-100)
    {
        pwm = -100;
    }
    return pwm;
}
void Drive::attach(DRV8833 driver1, DRV8833 driver2)
{
    this->driver1 = driver1;
    this->driver2 = driver2;
}

const float R = 0.030;
const float L = 0.078;
const float W = 0.146;

// Converts vx,vy,wz into wheel rotational velocity, a short burst at full speed is provided to overcome initial resistance
void Drive::drive(float vx,float vy,float wz)
{
    float fl = ((-(L+W)*wz)+vx-vy)/R;
    float fr = (((L+W)*wz)+vx+vy)/R;
    float br = (((L+W)*wz)+vx-vy)/R;
    float bl = ((-(L+W)*wz)+vx+vy)/R;
    int fl_pwm = vel2pwm(fl);
    int fr_pwm = vel2pwm(fr);
    int br_pwm = vel2pwm(br);
    int bl_pwm = vel2pwm(bl); 

    if (fl>0)
    {
        if (speeds[0] == 0)
        {
            driver1.forward(0,100);
            delay(10);
        }
        driver1.forward(0,fl_pwm);
    }
    else if (fl==0)
    {
        driver1.forward(0,0);
    }
    else
    {
        if (speeds[0]==0)
        {
            driver1.reverse(0,100);
            delay(10);
        }
        driver1.reverse(0,-fl_pwm);
    }


    if (fr>0)
    {
        if (speeds[1]==0)
        {
            driver2.forward(0,100);
            delay(10);
        }
        driver2.forward(0,fr_pwm);
    }
    else if (fr==0)
    {
        driver2.forward(0,0);
    }
    else
    {
        if (speeds[1]==0)
        {
            driver2.reverse(0,100);
            delay(10);
        }
        driver2.reverse(0,-fr_pwm);
    }


    if (bl>0)
    {
        if (speeds[3]==0)
        {
            driver1.forward(1,100);
            delay(10);
        }
        driver1.forward(1,bl_pwm-1);
    }
    else if (bl==0)
    {
        driver1.forward(1,0);
    }
    else
    {
        if (speeds[3]==0)
        {
            driver1.reverse(1,100);
            delay(10);
        }
        driver1.reverse(1,-bl_pwm-1);
    }


    if (br>0)
    {
        if (speeds[2]==0)
        {
            driver2.forward(1,100);
            delay(10);
        }
        driver2.forward(1,br_pwm);
    }
    else if (br==0)
    {
        driver2.forward(1,0);
    }
    else
    {
        if (speeds[2]==0)
        {
            driver2.reverse(1,100);
            delay(10);
        }
        driver2.reverse(1,-br_pwm);
    }
    speeds[0] = fl;
    speeds[1] = fr;
    speeds[2] = br;
    speeds[3] = bl;
}

void Drive::stop()
{
    driver1.forward(0,0);
    driver1.forward(1,0);
    driver2.forward(0,0);
    driver2.forward(1,0);
    driver1.sleep();
    driver2.sleep();
}

