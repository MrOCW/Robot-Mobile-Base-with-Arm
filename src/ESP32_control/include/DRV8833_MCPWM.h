#ifndef DRV8833_MCPWM_H
#define DRV8833_MCPWM_H

#include "Arduino.h"

#ifndef ESP32
#error "this library is only for ESP32"
#endif

class DRV8833 {
public:
	// Fields:
	uint16_t mMotorSpeed[2] = {0, 0};
	boolean mMotorForward[2] = {true, true};

	// Methods:

	// Attach two motors
	void attach(uint8_t driver, uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3, uint8_t gpioIn4,
	                  uint8_t gpioSleep, uint32_t frequencyHz = 1000);
	uint8_t get_driver();
	uint8_t get_sleep_pin();
	// Set speed -> PWM duty in the range 0-100
	void forward(uint8_t motor, uint8_t speed);
	void reverse(uint8_t motor, uint8_t speed);
	// Stop specific motor
	void sleep();

	uint8_t getMotorSpeed(uint8_t motor);
	boolean isMotorForward(uint8_t motor);
	boolean isMotorStopped(uint8_t motor);

private:
	// Fields:
	boolean mMotorAttached_[2] = {false, false};
	uint8_t sleep_pin;
	uint8_t driver;
	// Methods:
	void setMotor_(uint8_t motor, int8_t speed);
	boolean isMotorValid_(uint8_t motor);
};

#endif //DRV8833_MCPWM
