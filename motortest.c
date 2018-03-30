#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

void setupADC();
uint8_t readADC(int);
void setRightMotorSpeed(int);
void setLeftMotorSpeed(int);
void PIDcontrol(double *kp, double *kd, int *last_error, int);
void setBaseSpeed(int *base_speed, int);
int getCurrentPosition();

int main() {

	setupADC();

	// Make pin OC0A/B7 an output
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
	uint8_t sensor_array[8];

	while(1) {
		int i = 0;
		// for (i; i < 8; i++) {
		// 	sensor_array[i] = readADC(i);
		// }
		sensor_array[0] = readADC(0);
		if (sensor_array[0] > 128) {
			PORTB |= (1 << 5);
		}
		else {
			PORTB &= ~(1 << 5);
		}

		// OCR0A = 51;
		// OCR1A = 13107;
		// _delay_ms(3000);
		// OCR0A = 255;
		// OCR1A = 65535;
		// _delay_ms(3000);
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

void PIDcontrol(double *kp, double *kd, int *last_error, int base_speed) {
	int target_pos = 0;
	// Get the current position from 0
	int current_pos = getCurrentPosition();

	// Calculate error
	int error = target_pos - current_pos;

	// Calculate Derivative component
	int derivative = error - *last_error;

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

void setupADC() {
	// Set up ADC (internal ref, left-adjusted, 128 prescaler)
	// ADC0 active by default
	ADMUX |= (1<<REFS1)|(1<<REFS0)|(1<<ADLAR);
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	ADCSRA |= (1 << ADSC);
}

uint8_t readADC(int input_channel) {
	switch(input_channel) { // Set which ADC channel to use
		case 0:
			ADMUX = 0b11100100; //ADC4
			break;
		case 1:
			ADMUX = 0b11100101; //ADC5
			break;
		case 2:
			ADMUX = 0b11100110; //ADC6
			break;
		case 3:
			ADMUX = 0b11100111; //ADC7
			break;
		case 4:
			ADMUX = 0b11110011; //ADC11
			break;
		case 5:
			ADMUX = 0b11110010; //ADC10
			break;
		case 6:
			ADMUX = 0b11110001; //ADC9
			break;
		case 7:
			ADMUX = 0b11110000; //ADC8
	}

	ADCSRA |= (1 << ADSC); // Manually start conversion

	while(ADCSRA & (1 << ADSC)); // Wait until conversion is complete
	return (ADCH); // Return ADC output
}