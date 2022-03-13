#include "Arduino.h"
#include "DRV8833_MCPWM.h"
class Drive
{
public:
    void attach(DRV8833 driver1, DRV8833 driver2);

    void forward(uint8_t speed);
    void reverse(uint8_t speed);

    void strafe_left(uint8_t speed);
    void strafe_right(uint8_t speed);

    void rotate_ccw(uint8_t speed);
    void rotate_cw(uint8_t speed);

    void forward_left(uint8_t speed);
    void forward_right(uint8_t speed);
    void strafe_forward_left(uint8_t speed);
    void strafe_forward_right(uint8_t speed);

    void reverse_left(uint8_t speed);
    void reverse_right(uint8_t speed);
    void strafe_reverse_left(uint8_t speed);
    void strafe_reverse_right(uint8_t speed);
    
    void stop();

    void drive(float vx,float vy,float wz);
    float speeds[4];

private:
    DRV8833 driver1;
    DRV8833 driver2;
};