#include "drive.h"
#include "Arduino.h"
#include "DRV8833_MCPWM.h"

void Drive::attach(DRV8833 driver1, DRV8833 driver2)
{
    this->driver1 = driver1;
    this->driver2 = driver2;
}

float R = 0.030;
float L = 0.078;
float W = 0.146;

// Converts vx,vy,wz into wheel rotational velocity, a short burst at full speed is provided to overcome initial resistance
void Drive::drive(float vx,float vy,float wz)
{
    float fl = (((-L-W)*wz)+vx-vy)/R;
    float fr = (((L+W)*wz)+vx+vy)/R;
    float br = (((L+W)*wz)+vx-vy)/R;
    float bl = (((-L-W)*wz)+vx+vy)/R;
    
    if (fl>0)
    {
        if (speeds[0] == 0)
        {
            driver1.forward(0,100);
            delay(7);
        }
        driver1.forward(0,40);
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
            delay(5);
        }
        driver1.reverse(0,40);
    }


    if (fr>0)
    {
        if (speeds[1]==0)
        {
            driver2.forward(0,100);
            delay(5);
        }
        driver2.forward(0,40);
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
            delay(5);
        }
        driver2.reverse(0,40);
    }


    if (bl>0)
    {
        if (speeds[3]==0)
        {
            driver1.forward(1,100);
            delay(5);
        }
        driver1.forward(1,40);
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
            delay(5);
        }
        driver1.reverse(1,40);
    }


    if (br>0)
    {
        if (speeds[2]==0)
        {
            driver2.forward(1,100);
            delay(5);
        }
        driver2.forward(1,40);
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
            delay(5);
        }
        driver2.reverse(1,40);
    }
    speeds[0] = fl;
    speeds[1] = fr;
    speeds[2] = br;
    speeds[3] = bl;
}

// void Drive::forward(uint8_t speed)
// {
//     driver1.forward(0,speed);
//     driver1.forward(1,speed);
//     driver2.forward(0,speed);
//     driver2.forward(1,speed);
// }

// void Drive::reverse(uint8_t speed)
// {
//     driver1.reverse(0,speed);
//     driver1.reverse(1,speed);
//     driver2.reverse(0,speed);
//     driver2.reverse(1,speed);
// }

// void Drive::strafe_left(uint8_t speed)
// {
//     driver1.reverse(0,speed);
//     driver1.forward(1,speed);
//     driver2.forward(0,speed);
//     driver2.reverse(1,speed);
// }

// void Drive::strafe_right(uint8_t speed)
// {
//     driver1.forward(0,speed);
//     driver1.reverse(1,speed);
//     driver2.reverse(0,speed);
//     driver2.forward(1,speed);
// }

// void Drive::rotate_ccw(uint8_t speed)
// {
//     driver1.reverse(0,speed);
//     driver1.reverse(1,speed);
//     driver2.forward(0,speed);
//     driver2.forward(1,speed);
// }

// void Drive::rotate_cw(uint8_t speed)
// {
//     driver1.forward(0,speed);
//     driver1.forward(1,speed);
//     driver2.reverse(0,speed);
//     driver2.reverse(1,speed);
// }

// void Drive::forward_left(uint8_t speed)
// {
//     driver1.forward(0,speed);
//     driver1.forward(1,speed);
//     driver2.forward(0,speed+15);
//     driver2.forward(1,speed+15);
// }

// void Drive::forward_right(uint8_t speed)
// {
//     driver1.forward(0,speed+15);
//     driver1.forward(1,speed+15);
//     driver2.forward(0,speed);
//     driver2.forward(1,speed);
// }

// void Drive::strafe_forward_left(uint8_t speed)
// {
//     driver1.forward(1,speed);
//     driver2.forward(0,speed);
// }

// void Drive::strafe_forward_right(uint8_t speed)
// {
//     driver1.forward(0,speed);
//     driver2.forward(1,speed);
// }

// void Drive::reverse_left(uint8_t speed)
// {
//     driver1.reverse(0,speed);
//     driver1.reverse(1,speed);
//     driver2.reverse(0,speed+15);
//     driver2.reverse(1,speed+15);
// }

// void Drive::reverse_right(uint8_t speed)
// {
//     driver1.reverse(0,speed+15);
//     driver1.reverse(1,speed+15);
//     driver2.reverse(0,speed);
//     driver2.reverse(1,speed);
// }

// void Drive::strafe_reverse_left(uint8_t speed)
// {
//     driver1.reverse(0,speed);
//     driver2.reverse(1,speed);
// }

// void Drive::strafe_reverse_right(uint8_t speed)
// {
//     driver1.reverse(1,speed);
//     driver2.reverse(0,speed);
// }

void Drive::stop()
{
    driver1.forward(0,0);
    driver1.forward(1,0);
    driver2.forward(0,0);
    driver2.forward(1,0);
    driver1.sleep();
    driver2.sleep();
}

