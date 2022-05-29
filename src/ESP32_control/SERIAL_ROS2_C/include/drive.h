#include "driver/mcpwm.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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


void init_mcpwm(uint8_t driver0gpioIn1, uint8_t driver0gpioIn2, uint8_t driver0gpioIn3, uint8_t driver0gpioIn4, uint8_t driver0gpioSleep,
        uint8_t driver1gpioIn1, uint8_t driver1gpioIn2, uint8_t driver1gpioIn3, uint8_t driver1gpioIn4, uint8_t driver1gpioSleep,
        uint32_t frequencyHz)
{
    gpio_set_direction(driver0gpioSleep, GPIO_MODE_OUTPUT);
    gpio_set_level(driver0gpioSleep, 1);
    gpio_set_direction(driver1gpioSleep, GPIO_MODE_OUTPUT);
    gpio_set_level(driver1gpioSleep, 1);

    // Initial MCPWM configuration
    mcpwm_config_t cfg;
    cfg.frequency = frequencyHz;
    cfg.cmpr_a = 0;
    cfg.cmpr_b = 0;
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, driver0gpioIn1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, driver0gpioIn2);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, driver0gpioIn3);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, driver0gpioIn4);

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg);



    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, driver1gpioIn1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, driver1gpioIn2);

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, driver1gpioIn3);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, driver1gpioIn4);

    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &cfg);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &cfg);
}

void forward(int speed);
void reverse(int speed);

void strafe_left(int speed);
void strafe_right(int speed);

void rotate_ccw(int speed);
void rotate_cw(int speed);

void forward_left(int speed);
void forward_right(int speed);
void strafe_forward_left(int speed);
void strafe_forward_right(int speed);

void reverse_left(int speed);
void reverse_right(int speed);
void strafe_reverse_left(int speed);
void strafe_reverse_right(int speed);

void stop();

const float R = 0.030;
const float L = 0.0786;
const float W = 0.1265;
float speeds[4] = {0,0,0,0};
const uint8_t MIN_PWM = 40;
void drive(float vx, float vy, float wz)
{
    float fl = ((-(L+W)*wz)+vx-vy)/R;
    float fr = (((L+W)*wz)+vx+vy)/R;
    float br = (((L+W)*wz)+vx-vy)/R;
    float bl = ((-(L+W)*wz)+vx+vy)/R;
    int fl_pwm = vel2pwm(fl);
    int fr_pwm = vel2pwm(fr);
    int br_pwm = vel2pwm(br);
    int bl_pwm = vel2pwm(bl); 
    
    if (fl_pwm<0)
        fl_pwm = abs(fl_pwm);
    if (fr_pwm<0)
        fr_pwm = abs(fr_pwm);
    if (br_pwm<0)
        br_pwm = abs(br_pwm);
    else if (br_pwm>0)
        br_pwm += 5;
    if (bl_pwm<0)
        bl_pwm = abs(bl_pwm);

    // MIN_PWM
    if (fl_pwm<MIN_PWM && fl_pwm != 0)
        fl_pwm = MIN_PWM;
    if (fr_pwm<MIN_PWM && fr_pwm != 0)
        fr_pwm = MIN_PWM;
    if (br_pwm<MIN_PWM && br_pwm != 0)
        br_pwm = MIN_PWM+5;
    if (bl_pwm<MIN_PWM && bl_pwm != 0)
        bl_pwm = MIN_PWM;

    if (br_pwm>100)
        br_pwm = 100;
    printf("FL PWM: %d\n",fl_pwm);
    printf("FR PWM: %d\n",fr_pwm);
    printf("BL PWM: %d\n",bl_pwm);
    printf("BR PWM: %d\n",br_pwm);
    if (fl>0)
    {
        if (speeds[0] == 0)
        {
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, fl_pwm);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else if (fl==0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        if (speeds[0]==0)
        {
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, fl_pwm);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }


    if (fr>0)
    {
        if (speeds[1]==0)
        {
        	mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, fr_pwm);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else if (fr==0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    else
    {
        if (speeds[1]==0)
        {
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, fr_pwm);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }


    if (bl>0)
    {
        if (speeds[3]==0)
        {
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, bl_pwm);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else if (bl==0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    else
    {
        if (speeds[3]==0)
        {
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, bl_pwm);
		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }


    if (br>0)
    {
        if (speeds[2]==0)
        {
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, br_pwm);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else if (br==0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        if (speeds[2]==0)
        {
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, 100);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, -br_pwm);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    speeds[0] = fl;
    speeds[1] = fr;
    speeds[2] = br;
    speeds[3] = bl;
}

