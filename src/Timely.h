//
// Timely
//
// An Arduino library to conveniently set GPIO pins
//

#ifndef TIMELY_H
#define TIMELY_H

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//
// Defines for blinking the LED
//

// Arduino Mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define BLINKLED       13
#define BLINKLED_ON()  (PORTB |= B10000000)
#define BLINKLED_OFF() (PORTB &= B01111111)

// Sanguino
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
#define BLINKLED       0
#define BLINKLED_ON()  (PORTD |= B00000001)
#define BLINKLED_OFF() (PORTD &= B11111110)

// Readbear Wifi Mini
#elif defined(__CC3200R1MXRGCR__)
#define BLINKLED       13
#define PORTD(pin)     (HWREG(GPIOA3_BASE + (GPIO_O_GPIO_DATA + (BV(pin) << 2))))
#define BLINKLED_ON()  (PORTD(6) |= B01000000)
#define BLINKLED_OFF() (PORTD(6) &= B10111111)

#else
#define BLINKLED       13
#define BLINKLED_ON()  (PORTB |= B00100000)
#define BLINKLED_OFF() (PORTB &= B11011111)
#endif

//
// CPU Frequency
//

// Main Arduino
#if defined(F_CPU)
#define SYSCLOCK F_CPU

// Default Arduino
#else
#define SYSCLOCK 16000000UL
#endif

//
// Timer definitions
//

// Arduino Mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
const static char *kAvailableTimers[] = { "TIMER1", "TIMER2", "TIMER3", "TIMER4", "TIMER5" };
//#define USE_TIMER1
#define USE_TIMER2
//#define USE_TIMER3
//#define USE_TIMER4
//#define USE_TIMER5

// Teensy 1.0
#elif defined(__AVR_AT90USB162__)
const static char *kAvailableTimers[] = { "TIMER1" };
#define USE_TIMER1

// Teensy 2.0, Arduino Leonardo, Yún, Pro, Pro Mini, Esplora, Micro, Leonardo, etc
#elif defined(__AVR_ATmega32U4__)
const static char *kAvailableTimers[] = { "TIMER1", "TIMER3", "TIMER4_HS" };
//#define USE_TIMER1
//#define USE_TIMER3
#define USE_TIMER4_HS

// Teensy 3.0 / Teensy 3.1
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
const static char *kAvailableTimers[] = { "TIMER_CMT" };
#define USE_TIMER_CMT

// Teensy-LC
#elif defined(__MKL26Z64__)
const static char *kAvailableTimers[] = { "TIMER_TPM1" };
#define USE_TIMER_TPM1

// Teensy++ 1.0 & 2.0
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
const static char *kAvailableTimers[] = { "TIMER1", "TIMER2", "TIMER3" };
//#define USE_TIMER1
#define USE_TIMER2
//#define USE_TIMER3

// Sanguino
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
const static char *kAvailableTimers[] = { "TIMER1", "TIMER2" };
//#define USE_TIMER1
#define USE_TIMER2

// Atmega8
#elif defined(__AVR_ATmega8P__) || defined(__AVR_ATmega8__)
const static char *kAvailableTimers[] = { "TIMER1" };
#define USE_TIMER1

// ATtiny84
#elif defined(__AVR_ATtiny84__)
const static char *kAvailableTimers[] = { "TIMER1" };
#define USE_TIMER1

// ATtiny85
#elif defined(__AVR_ATtiny85__)
const static char *kAvailableTimers[] = { "TINY0" };
#define USE_TIMER_TINY0

// Arduino Duemilanove, Diecimila, Uno, LilyPad, Mini, Fio, Nano, etc
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
const static char *kAvailableTimers[] = { "TIMER1", "TIMER2" };
//#define USE_TIMER1
#define USE_TIMER2

// Readbear Wifi Mini
#elif defined(__CC3200R1MXRGCR__)
const static char *kAvailableTimers[] = { "TIMER2", "TIMER3", "TIMER4", "TIMER5" };
#define USE_TIMER_2
//#define USE_TIMER_3
//#define USE_TIMER_4
//#define USE_TIMER_5

#else
#error "Architecture constant not recognised by Timely. No timer will be configured.\n"
#endif

//
// Timer setup
//

// AVR TIMER1 (16 bits) - Teensy 1.0, Teensy 2.0, Teensy++ 1.0 & 2.0, Sanguino, Atmega8, ATtiny84, Arduino Duemilanove,
// Diecimila, LilyPad, Mega, Mini, Fio, Nano, etc
#if defined(USE_TIMER1)

#define TIMER_ENABLE_PWM  (TCCR1A |= _BV(COM1A1))
#define TIMER_DISABLE_PWM (TCCR1A &= ~(_BV(COM1A1)))

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	const uint16_t pwmval = SYSCLOCK / 2000 / (val);                                                                    \
	TCCR1A                = _BV(WGM11);                                                                                 \
	TCCR1B                = _BV(WGM13) | _BV(CS10);                                                                     \
	ICR1                  = pwmval;                                                                                     \
	OCR1A                 = pwmval / 3;                                                                                 \
})

#if defined(CORE_OC1A_PIN)
#define TIMER_PWM_PIN CORE_OC1A_PIN // Teensy
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TIMER_PWM_PIN 11            // Arduino Mega
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
#define TIMER_PWM_PIN 13            // Sanguino
#elif defined(__AVR_ATtiny84__)
#define TIMER_PWM_PIN 6
#else
#define TIMER_PWM_PIN 9             // Arduino Duemilanove, Diecimila, LilyPad, etc
#endif

// AVR TIMER2 (8 bits) - Teensy++ 1.0 & 2.0, Sanguino, Arduino Duemilanove, Diecimila, LilyPad, Mega, Mini, Fio, Nano,
// etc
#elif defined(USE_TIMER2)

#define TIMER_ENABLE_PWM  (TCCR2A |= _BV(COM2B1))
#define TIMER_DISABLE_PWM (TCCR2A &= ~(_BV(COM2B1)))

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	const uint8_t pwmval = SYSCLOCK / 2000 / (val);                                                                     \
	TCCR2A               = _BV(WGM20);                                                                                  \
	TCCR2B               = _BV(WGM22) | _BV(CS20);                                                                      \
	OCR2A                = pwmval;                                                                                      \
	OCR2B                = pwmval / 3;                                                                                  \
})

#if defined(CORE_OC2B_PIN)
#define TIMER_PWM_PIN CORE_OC2B_PIN // Teensy
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TIMER_PWM_PIN 9             // Arduino Mega
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
#define TIMER_PWM_PIN 14            // Sanguino
#else
#define TIMER_PWM_PIN 3             // Arduino Duemilanove, Diecimila, LilyPad, etc
#endif

// AVR TIMER3 (16 bits) - Teensy 2.0, Teensy++ 1.0 & 2.0, Arduino Mega
#elif defined(USE_TIMER3)

#define TIMER_ENABLE_PWM  (TCCR3A |= _BV(COM3A1))
#define TIMER_DISABLE_PWM (TCCR3A &= ~(_BV(COM3A1)))

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	const uint16_t pwmval = SYSCLOCK / 2000 / (val);                                                                    \
	TCCR3A                = _BV(WGM31);                                                                                 \
	TCCR3B                = _BV(WGM33) | _BV(CS30);                                                                     \
	ICR3                  = pwmval;                                                                                     \
	OCR3A                 = pwmval / 3;                                                                                 \
})

#if defined(CORE_OC3A_PIN)
#define TIMER_PWM_PIN  CORE_OC3A_PIN // Teensy
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TIMER_PWM_PIN 5              // Arduino Mega
#else
#error "Please add OC3A pin number here\n"
#endif

// AVR TIMER4 (16 bits) - Arduino Mega
#elif defined(USE_TIMER4)

#define TIMER_ENABLE_PWM  (TCCR4A |= _BV(COM4A1))
#define TIMER_DISABLE_PWM (TCCR4A &= ~(_BV(COM4A1)))

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	const uint16_t pwmval = SYSCLOCK / 2000 / (val);                                                                    \
	TCCR4A                = _BV(WGM41);                                                                                 \
	TCCR4B                = _BV(WGM43) | _BV(CS40);                                                                     \
	ICR4                  = pwmval;                                                                                     \
	OCR4A                 = pwmval / 3;                                                                                 \
})

#if defined(CORE_OC4A_PIN)
#define TIMER_PWM_PIN CORE_OC4A_PIN
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TIMER_PWM_PIN 6 // Arduino Mega
#else
#error "Please add OC4A pin number here\n"
#endif

// AVR TIMER4 (10 bits, high speed) - Teensy 2.0
#elif defined(USE_TIMER4_HS)

#define TIMER_ENABLE_PWM  (TCCR4A |= _BV(COM4A1))
#define TIMER_DISABLE_PWM (TCCR4A &= ~(_BV(COM4A1)))

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	const uint16_t pwmval = SYSCLOCK / 2000 / (val);                                                                    \
	TCCR4A                = (1<<PWM4A);                                                                                 \
	TCCR4B                = _BV(CS40);                                                                                  \
	TCCR4C                = 0;                                                                                          \
	TCCR4D                = (1<<WGM40);                                                                                 \
	TCCR4E                = 0;                                                                                          \
	TC4H                  = pwmval >> 8;                                                                                \
	OCR4C                 = pwmval;                                                                                     \
	TC4H                  = (pwmval / 3) >> 8;                                                                          \
	OCR4A                 = (pwmval / 3) & 255;                                                                         \
})

#if defined(CORE_OC4A_PIN)
#define TIMER_PWM_PIN CORE_OC4A_PIN // Teensy
#elif defined(__AVR_ATmega32U4__)
#define TIMER_PWM_PIN 13            // Leonardo
#else
#error "Please add OC4A pin number here\n"
#endif

// AVR TIMER5 (16 bits) - Arduino Mega
#elif defined(USE_TIMER5)

#define TIMER_ENABLE_PWM  (TCCR5A |= _BV(COM5A1))
#define TIMER_DISABLE_PWM (TCCR5A &= ~(_BV(COM5A1)))

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	const uint16_t pwmval = SYSCLOCK / 2000 / (val);                                                                    \
	TCCR5A                = _BV(WGM51);                                                                                 \
	TCCR5B                = _BV(WGM53) | _BV(CS50);                                                                     \
	ICR5                  = pwmval;                                                                                     \
	OCR5A                 = pwmval / 3;                                                                                 \
})

#if defined(CORE_OC5A_PIN)
#define TIMER_PWM_PIN CORE_OC5A_PIN
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TIMER_PWM_PIN 46 // Arduino Mega
#else
#error "Please add OC5A pin number here\n"
#endif

// Special carrier modulator timer - Teensy 3.0 / Teensy 3.1
#elif defined(USE_TIMER_CMT)

#define TIMER_ENABLE_PWM do {                                                                                         \
	CORE_PIN5_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE | PORT_PCR_SRE;                                                   \
} while(0)

#define TIMER_DISABLE_PWM do {                                                                                        \
	CORE_PIN5_CONFIG = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE;                                                   \
} while(0)

#if (F_BUS == 48000000)
#define CMT_PPS_VAL 5
#else
#define CMT_PPS_VAL 2
#endif

#define TIMER_CONFIG_KHZ(val) ({ 	                                                                                    \
	SIM_SCGC4 |= SIM_SCGC4_CMT;                                                                                         \
	SIM_SOPT2 |= SIM_SOPT2_PTD7PAD;                                                                                     \
	CMT_PPS    = CMT_PPS_VAL;                                                                                           \
	CMT_CGH1   = 2667 / val;                                                                                            \
	CMT_CGL1   = 5333 / val;                                                                                            \
	CMT_CMD1   = 0;                                                                                                     \
	CMT_CMD2   = 30;                                                                                                    \
	CMT_CMD3   = 0;                                                                                                     \
	CMT_CMD4   = 0;                                                                                                     \
	CMT_OC     = 0x60;                                                                                                  \
	CMT_MSC    = 0x01;                                                                                                  \
})

#define TIMER_PWM_PIN 5

// Teensy-LC
#elif defined(USE_TIMER_TPM1)

#define TIMER_ENABLE_PWM  CORE_PIN16_CONFIG = PORT_PCR_MUX(3)|PORT_PCR_DSE|PORT_PCR_SRE
#define TIMER_DISABLE_PWM CORE_PIN16_CONFIG = PORT_PCR_MUX(1)|PORT_PCR_SRE

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	SIM_SCGC6 |= SIM_SCGC6_TPM1;                                                                                        \
	FTM1_SC    = 0;                                                                                                     \
	FTM1_CNT   = 0;                                                                                                     \
	FTM1_MOD   = (F_PLL/2000) / val - 1;                                                                                \
	FTM1_C0V   = (F_PLL/6000) / val - 1;                                                                                \
	FTM1_SC    = FTM_SC_CLKS(1) | FTM_SC_PS(0);                                                                         \
})

#define TIMER_PWM_PIN 16

// ATtiny85 (8 bits)
#elif defined(USE_TIMER_TINY0)

#define TIMER_ENABLE_PWM  (TCCR0A |= _BV(COM0B1))
#define TIMER_DISABLE_PWM (TCCR0A &= ~(_BV(COM0B1)))

#define TIMER_CONFIG_KHZ(val) ({                                                                                      \
	const uint8_t pwmval = SYSCLOCK / 2000 / (val);                                                                     \
	TCCR0A               = _BV(WGM00);                                                                                  \
	TCCR0B               = _BV(WGM02) | _BV(CS00);                                                                      \
	OCR0A                = pwmval;                                                                                      \
	OCR0B                = pwmval / 3;                                                                                  \
})

// ATtiny85
#define TIMER_PWM_PIN 1

#endif

//
// Timing Functions
//

#define RAD(degree, denominator) (degree * M_PI / denominator)

/**
 * Timing function definition taking several
 * arguments used as control points for a
 * tween operation.
 *
 * Dividing the position by the duration is
 * the percentage of completion of the tween.
 *
 * @type {timingFunction}
 *
 * @param {float} position
 *   The current "position" between the start and end of the tween. This must be the same unit as the duration.
 *
 * @param {float} from
 *   The initial value of the tween.
 *
 * @param {float} displacement
 *   The displacement between the initial and final value of the tween.
 *
 * @param {unsigned long} duration
 *   The total duration of the tween
 *
 * @return {float}
 *   Tweened value.
 */
typedef float (*timingFunction)(float, float, float, unsigned long);

class Timely {
protected:

  /**
   * To be used with generating a carrier signal,
   * this specifies how long to keep the PWM pin
   * high.
   *
   * @method mark
   *
   * @param {unsigned long} uSec
   *   The duration in microseconds to keep the pin high.
   */
  void mark(unsigned long uSec);

  /**
   * To be used with generating a carrier signal,
   * this specifies how long to keep the PWM pin
   * low.
   *
   * @method space
   *
   * @param {unsigned long} uSec
   *   The duration in microseconds to keep the pin low.
   */
  void space(unsigned long uSec);

  /**
   * Enable the selected timer to run at a specific
   * frequency. This allows a PWM pin to blink on
   * and off at a constant rate.
   *
   * @method enableTimer
   *
   * @param {unsigned int} frequency
   *   The frequency to send the carrier signal (in kHz).
   */
  void enableTimer(unsigned int frequency);

public:

  /**
   * Send a raw signal over a specific frequency. This
   * will typically be used with IR but is not restricted
   * to it.
   *
   * @method sendRaw
   *
   * @param {unsigned int*} buffer
   *   The carrier signal data.
   *
   * @param {unsigned int} length
   *   Number of bits in the carrrier signal.
   *
   * @param {unsigned int} frequency
   *   The frequency to send the carrier signal (in kHz).
   */
  void sendRaw(unsigned int buffer[], unsigned int length, unsigned int frequency);

  /**
   * Abstract, method defining how to send a carrier
   * signal. Typically for IR but not restricted to
   * it.
   *
   * @method send
   *
   * @param {unsigned long} data
   *   The carrier signal data.
   *
   * @param {unsigned int} length
   *   Number of bits in the carrrier signal.
   */
  virtual void send(unsigned long data, unsigned int length) {};

  /**
   * Set the duty cycle of a PWM pin for rapid blink
   * effect.
   *
   * @method blink
   *
	 * @param {unsigned long} uSec
	 *   The duration of time to blink the pin.
	 *
   * @param {unsigned int} frequency
   *   The frequency to blink the pin (in kHz).
   */
  void blink(unsigned long uSec, unsigned int frequency);

  /**
   * Fade in and out a PWM pin from one value to
   * another using in linear time.
   *
   * @method fade
   *
   * @param {unint8_t} from
   *   The value to fade from.
   *
   * @param {uint8_t} to
   *   The value to fade to.
	 *
	 * @param {unsigned long} uSec
	 *   The duration of time to fade from one value to another.
   */
  void fade(uint8_t from, uint8_t to, unsigned long uSec);

  /**
   * Fade in and out a PWM pin from one value to
   * another using a timing function.
   *
   * @method fade
   *
   * @param {unsigned long} from
   *   The value to fade from.
   *
   * @param {unsinged long} to
   *   The value to fade to.
	 *
	 * @param {unsigned long} uSec
	 *   The duration of time to fade from one value to another.
	 *
	 * @param {timingFunction} func
	 *   Function pointer.
   */
  void fade(uint8_t from, uint8_t to, unsigned long uSec, timingFunction func);

  /**
   * Return an array of available timers.
   *
   * @method availableTimers
   *
   * @return {char**}
   *   An array of avaiable timer numbers. Will not return which ones are in use.
   */
  const static char** availableTimers();
};

#endif
