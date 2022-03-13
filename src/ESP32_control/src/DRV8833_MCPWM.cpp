#include "DRV8833_MCPWM.h"

#include <cmath>

#include "Arduino.h"
#include "driver/mcpwm.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

uint8_t DRV8833::get_driver()
{
	return driver;
}
uint8_t DRV8833::get_sleep_pin()
{
	return sleep_pin;
}
void DRV8833::attach(uint8_t driver, uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3,
                                     uint8_t gpioIn4, uint8_t gpioSleep, uint32_t frequencyHz)
{
	this->driver = driver;
	sleep_pin = gpioSleep;
	if (driver == 0)
	{
		//Serial.println("Initializing driver 0 - MCPWM Unit 0 with Timers 0 and 1\n");
		mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpioIn1);
		mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, gpioIn2);

		// Indicate the motor 0 is attached.
		this->mMotorAttached_[0] = true;

		mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, gpioIn3);
		mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, gpioIn4);

		// Indicate the motor 1 is attached.
		this->mMotorAttached_[1] = true;


		// Initial MCPWM configuration
		mcpwm_config_t cfg;
		cfg.frequency = frequencyHz;
		cfg.cmpr_a = 0;
		cfg.cmpr_b = 0;
		cfg.counter_mode = MCPWM_UP_COUNTER;
		cfg.duty_mode = MCPWM_DUTY_MODE_0;

		mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
		mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg);
	}
	else if (driver == 1)
	{
		//Serial.println("Initializing driver 1 - MCPWM Unit 1 with Timers 0 and 1\n");
		mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, gpioIn1);
		mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, gpioIn2);

		// Indicate the motor 0 is attached.
		this->mMotorAttached_[0] = true;

		mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, gpioIn3);
		mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, gpioIn4);

		// Indicate the motor 1 is attached.
		this->mMotorAttached_[1] = true;


		// Initial MCPWM configuration
		mcpwm_config_t cfg;
		cfg.frequency = frequencyHz;
		cfg.cmpr_a = 0;
		cfg.cmpr_b = 0;
		cfg.counter_mode = MCPWM_UP_COUNTER;
		cfg.duty_mode = MCPWM_DUTY_MODE_0;

		// Configure PWM0A & PWM0B with above settings
		mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &cfg);

		// Configure PWM1A & PWM1B with above settings
		mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &cfg);
	}	
}

void DRV8833::forward(uint8_t motor, uint8_t speed) 
{
	if (get_driver()==0)
	{
		if (motor == 0)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

		}
		else if (motor == 1)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

		}
	}

	else if (get_driver()==1)
	{
		if (motor == 0)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		}
		else if (motor == 1)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		}
	}
	
	
}

void DRV8833::reverse(uint8_t motor, uint8_t speed)
{
	if (get_driver()==0)
	{
		if (motor == 0)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
		}
		else if (motor == 1)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
		}
	}

	else if (get_driver()==1)
	{
		if (motor == 0)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
		}
		else if (motor == 1)
		{
			mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
		}
	}
}


void DRV8833::sleep()
{
	mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
	mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
	mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
	mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
	mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
	mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
	mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
	mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
}

uint8_t DRV8833::getMotorSpeed(uint8_t motor)
{
	if (!isMotorValid_(motor))
		return false;

	return mMotorSpeed[motor];
}

boolean DRV8833::isMotorForward(uint8_t motor)
{
	if (!isMotorValid_(motor) || isMotorStopped(motor))
		return false;

	return mMotorForward[motor];
}

boolean DRV8833::isMotorStopped(uint8_t motor)
{
	if (!isMotorValid_(motor))
		return true;

	return (mMotorSpeed[motor] == 0);
}


boolean DRV8833::isMotorValid_(uint8_t motor)
{
	if (motor > 1)
		return false;

	return mMotorAttached_[motor];
}
