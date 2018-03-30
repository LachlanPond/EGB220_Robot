#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main() {
	// Make pin OC0A/B7 an output
	DDRB |= (1 << 7);
	DDRB |= (1 << 5);

	// Configure TCCR0A (Motor 2)
	// -- Clear OC0A on Compare Match, set OC0A at TOP
	// -- Clear OC0B on Compare Match, set OC0B at TOP
	// -- Fast PWM
	TCCR0A |= (1 << 7) | (1 << 5) | (1 << 1) | 1;

	// Configure TCCR0A (Motor 1)
	// -- Clear OC1A on Compare Match, set OC1A at TOP
	// -- Clear OC1B on Compare Match, set OC1B at TOP
	// -- Fast PWM 8-bit
	TCCR1A |= (1 << 7) | (1 << 5) | 1;

	// 256 (From prescalar)
	TCCR0B |= (1 << 2);
	TCCR1B |= (1 << 2) | (1 << 3);

	// Lets make the Duty Cycle 40%
	// The full cycle represents a count to 255, therefore:
	// -- OCR0 = DC x 255
	// -- OCR0 = 0.4 x 255 = 102
	// Set compare value
	// OCR0A = 102;

	// Setup some initial variables
	int integral = 0;
	int base_speed = 60;
	double kp = 0;
	double kd = 0;

	while(1) {
		OCR0A = 51;
		OCR1A = 13107;
		_delay_ms(3000);
		OCR0A = 255;
		OCR1A = 65535;
		_delay_ms(3000);
	}
}

// Set right motor speed using a percentage
void setRightMotorSpeed(int percentage) {
	if (percentage > 100) {
		percentage = 100;
	}
	else if (percentage < 20) {
		percentage = 20;
	}
	OCR1A = 65535 * (percentage * 0.01); 
}

// Set left motor speed using a percentage
void setLeftMotorSpeed(int percentage) {
	if (percentage > 100) {
		percentage = 100;
	}
	else if (percentage < 20) {
		percentage = 20;
	}
	OCR0A = 255 * (percentage * 0.01); 
}

void PIDcontrol(double *kp, double *kd, int *last_error) {
	int target_pos = 0;
	// Get the current position from 0
	int current_pos = getCurrentPosition();

	// Calculate error
	int error = target_pos - current_pos;

	// Calculate Derivative component
	int derivative = error - last_error;

	// Calculate Control Variable
	int control_variable = (*kp * error) + (*kd * derivative);

	setLeftMotorSpeed(base_speed + control_variable);
	setRightMotorSpeed(base_speed + control_variable);

	*last_error = error;
}

int getCurrentPosition() {
	// Read IR sensors and work out how far from 0 the robot is
}

void setBaseSpeed(int *base_speed, int speed) {
	*base_speed = speed;
}