#include "Timely.h"

#define DISPLACEMENT(to, from) (from > to ? to - from : from - to)

static float linear(float position, float from, float displacement, unsigned long duration) {
	return displacement * (position / duration) + from;
}

void Timely::mark(unsigned long uSec) {
  TIMER_ENABLE_PWM;

	if (uSec > 0) {
		delayMicroseconds(uSec);
	}
}

void Timely::space(unsigned long uSec) {
  TIMER_DISABLE_PWM;

	if (uSec > 0) {
		delayMicroseconds(uSec);
	}
}

void Timely::enableTimer(unsigned int frequency) {

	// When not sending PWM, we want it low.
	pinMode(TIMER_PWM_PIN, OUTPUT);
	digitalWrite(TIMER_PWM_PIN, LOW);

	// Set the duty cycle for the timer.
	TIMER_CONFIG_KHZ(frequency);
}

void Timely::sendRaw(unsigned int buffer[], unsigned int length, unsigned int frequency) {
  this->enableTimer(frequency);

  for (unsigned int i = 0; i < length; i++) {
	  if (i & 1) {
			this->space(buffer[i]);
		} else {
			this->mark(buffer[i]);
		}
  }

  this->space(0);
}

void Timely::blink(unsigned long uSec, unsigned int frequency) {
	this->enableTimer(frequency);
	this->mark(uSec);
	this->space(0);
}

void Timely::fade(uint8_t from, uint8_t to, unsigned long uSec) {
  this->fade(from, to, uSec, &linear);
}

void Timely::fade(uint8_t from, uint8_t to, unsigned long uSec, timingFunction func) {
	float displacement = DISPLACEMENT(to, from);
	float out = .0f;

	for (unsigned long position = .0f; position < uSec; position += 0.001) {
		out = func(position, from, displacement, uSec);
		this->enableTimer(out);
		this->mark(1000);
	}

	// If we are fading to zero then turn
	// off PWM.
	if (to == 0) {
		this->space(0);
	}
}

const char** Timely::availableTimers() {
  return kAvailableTimers;
}
