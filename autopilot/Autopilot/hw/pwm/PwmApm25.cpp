/*
 *       PwmApm25.cpp - Radio Control Library for Ardupilot Mega 2.0. Arduino
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       RC Input : PPM signal on IC4 pin
 *       RC Output : 11 Servo outputs (standard 20ms frame)
 *
 *       Methods:
 *               Init() : Initialization of interrupts an Timers
 *               OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
 *               InputCh(ch) : Read a channel input value.  ch=0..7
 *               GetState() : Returns the state of the input. 1 => New radio frame to process
 *                            Automatically resets when we call InputCh to read channels
 *
 */
#include "PwmApm25.hpp"
#include <infra/rtos/Task.hpp>

#include <avr/interrupt.h>
#include "Arduino.h"

#include <system/params/Nrd.hpp>

namespace hw {


#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(DESKTOP_BUILD)
 # error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile uint8_t PwmApm25::_radio_status = 0;
volatile uint16_t PwmApm25::_PWM_RAW[] = {
		2400,
		2400,
		2400,
		2400,
		2400,
		2400,
		2400,
		2400};

#define constrain(val,valMin,valMax) \
	( ((val)<(valMin)) ? (valMin) : ( ((val)>(valMax)) ? (valMax) : (val) ) )

/****************************************************
*   Input Capture Interrupt ICP5 => PPM signal read
****************************************************/
void PwmApm25::_timer5_capt_cb(void)
{
    static uint16_t prev_icr;
    static uint8_t frame_idx;
    uint16_t icr;
    uint16_t pwidth;

    icr = ICR5;
    // Calculate pulse width assuming timer overflow TOP = 40000
    if ( icr < prev_icr ) {
        pwidth = ( icr + 40000 ) - prev_icr;
    } else {
        pwidth = icr - prev_icr;
    }

    // Was it a sync pulse? If so, reset frame.
    if ( pwidth > 8000 ) {
        // pass through values if at least a minimum number of channels received
        if( frame_idx >= MIN_CHANNELS ) {
            _radio_status = 1;
//            _last_update = millis();
        }
        frame_idx=0;
    } else {
        // Save pulse into _PWM_RAW array.
        if ( frame_idx < APM25_PWM_NUM_TO_DEVICE ) {
            _PWM_RAW[ frame_idx++ ] = pwidth ;
        }
    }
    // Save icr for next call.
    prev_icr = icr;
}


// Constructors ////////////////////////////////////////////////////////////////

PwmApm25::PwmApm25()
: Pwm()
{
}

PwmApm25::~PwmApm25()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void PwmApm25::reset()
{
    // Disable Input Capture interrupt
    TIMSK5 &= ~(1<<ICIE5);

    // Disable all outputs
    disable_out(CH_1);
    disable_out(CH_2);
    disable_out(CH_3);
    disable_out(CH_4);
    disable_out(CH_5);
    disable_out(CH_6);
    disable_out(CH_7);
    disable_out(CH_8);
    disable_out(CH_9);
    disable_out(CH_10);
    disable_out(CH_11);
}

bool PwmApm25::initialize()
{

    // --------------------- TIMER1: OUT1 and OUT2 -----------------------
    digitalWrite(12,HIGH);  // pulling high before changing to output avoids a momentary drop of the pin to low because the ESCs have a pull-down resistor it seems
    digitalWrite(11,HIGH);
    pinMode(12,OUTPUT); // OUT1 (PB6/OC1B)
    pinMode(11,OUTPUT); // OUT2 (PB5/OC1A)
    digitalWrite(12,HIGH);  // pulling high before changing to output avoids a momentary drop of the pin to low because the ESCs have a pull-down resistor it seems
    digitalWrite(11,HIGH);

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR1.
    // CS11: prescale by 8 => 0.5us tick
    TCCR1A =((1<<WGM11));
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = 40000; // 0.5us tick => 50hz freq
    OCR1A = 0xFFFF; // Init OCR registers to nil output signal
    OCR1B = 0xFFFF;

    // --------------- TIMER4: OUT3, OUT4, and OUT5 ---------------------
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(6,HIGH);
    pinMode(8,OUTPUT); // OUT3 (PH5/OC4C)
    pinMode(7,OUTPUT); // OUT4 (PH4/OC4B)
    pinMode(6,OUTPUT); // OUT5 (PH3/OC4A)
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(6,HIGH);

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR4.
    // CS41: prescale by 8 => 0.5us tick
    TCCR4A =((1<<WGM41));
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    OCR4A = 0xFFFF; // Init OCR registers to nil output signal
    OCR4B = 0xFFFF;
    OCR4C = 0xFFFF;
    ICR4 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER3: OUT6, OUT7, and OUT8 ----------------------
    digitalWrite(3,HIGH);
    digitalWrite(2,HIGH);
    digitalWrite(5,HIGH);
    pinMode(3,OUTPUT); // OUT6 (PE5/OC3C)
    pinMode(2,OUTPUT); // OUT7 (PE4/OC3B)
    pinMode(5,OUTPUT); // OUT8 (PE3/OC3A)
    digitalWrite(3,HIGH);
    digitalWrite(2,HIGH);
    digitalWrite(5,HIGH);

    // WGM: 1 1 1 0. Clear timer on Compare, TOP is ICR3
    // CS31: prescale by 8 => 0.5us tick
    TCCR3A =((1<<WGM31));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
    OCR3A = 0xFFFF; // Init OCR registers to nil output signal
    OCR3B = 0xFFFF;
    OCR3C = 0xFFFF;
    ICR3 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER5: PPM INPUT, OUT10, and OUT11 ---------------
    // Init PPM input on Timer 5
    pinMode(48, INPUT); // PPM Input (PL1/ICP5)

    digitalWrite(45,HIGH);
    digitalWrite(44,HIGH);
    pinMode(45, OUTPUT); // OUT10 (PL4/OC5B)
    pinMode(44, OUTPUT); // OUT11 (PL5/OC5C)
    digitalWrite(45,HIGH);
    digitalWrite(44,HIGH);

    // WGM: 1 1 1 1. Fast PWM, TOP is OCR5A
    // COM all disabled.
    // CS51: prescale by 8 => 0.5us tick
    // ICES5: Input Capture on rising edge
    TCCR5A =((1<<WGM50)|(1<<WGM51));
    // Input Capture rising edge
    TCCR5B = ((1<<WGM53)|(1<<WGM52)|(1<<CS51)|(1<<ICES5));
    OCR5A = 40000; // 0.5us tick => 50hz freq. The input capture routine
                   // assumes this 40000 for TOP.

//    isr_reg->register_signal( ISR_REGISTRY_TIMER5_CAPT, _timer5_capt_cb );
    // Enable Input Capture interrupt
    TIMSK5 |= (1<<ICIE5);

    return true;
}

void PwmApm25::write(uint8_t idx, const uint16_t& pwmValue)
{

	switch (idx)
	{
	case CH_1:
		OCR1B=(pwmValue << 1);  // out1
		break;
	case CH_2:
		OCR1A=(pwmValue << 1);  // out1
		break;
	case CH_3:
		OCR4C=(pwmValue << 1);  // out1
		break;
	case CH_4:
		OCR4B=(pwmValue << 1);  // out1
		break;
	case CH_5:
		OCR4A=(pwmValue << 1);  // out1
		break;
	case CH_6:
		OCR3C=(pwmValue << 1);  // out1
		break;
	case CH_7:
		OCR3B=(pwmValue << 1);  // out1
		break;
	case CH_8:
		OCR3A=(pwmValue << 1);  // out1
		break;
	case CH_9:
		/* Do nothing */
		break;
	case CH_10:
		OCR5B=(pwmValue << 1);  // out1
		break;
	case CH_11:
		OCR5C=(pwmValue << 1);  // out1
		break;
	default:
		/* Do nothing */
		break;
	}
}

void PwmApm25::write(const pwm_t* pwmValues)
{
	/* Process inputs (i.e. convert inputs to PWM */
	OCR1B=(pwmValues[0] << 1);  // out1
	OCR1A=(pwmValues[1] << 1);  // out2
	OCR4C=(pwmValues[2] << 1);  // out3
	OCR4B=(pwmValues[3] << 1);  // out4
	OCR4A=(pwmValues[4] << 1);  // out5
	OCR3C=(pwmValues[5] << 1);  // out6
	OCR3B=(pwmValues[6] << 1);  // out7
	OCR3A=(pwmValues[7] << 1);  // out8
	/* Output 9 not implemented: rational ? */
	OCR5B=(pwmValues[9] << 1);  // out10
	OCR5C=(pwmValues[10] << 1);  // out11
}

void PwmApm25::read(uint8_t idx, uint16_t& pwmValues)
{
	uint8_t mask = (1<<idx);

	// Mask timer 5 interrupt
    uint8_t _timsk5 = TIMSK5;
    TIMSK5 &= ~(1<<ICIE5);

	if (_radio_status & mask && idx<APM25_PWM_NUM_FROM_DEVICE)
	{
		// value
		// Because timer runs at 0.5us we need to do value/2
		pwmValues = constrain((_PWM_RAW[idx] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
	}

	/* Signal as been read */
	_radio_status &= ~mask;     // Radio channel read

    //	Resume timer 5 interrupt
    TIMSK5 = _timsk5;
}

void PwmApm25::read(hw::pwm_t* pwmValues)
{
	// Mask timer 5 interrupt
    uint8_t _timsk5 = TIMSK5;
    TIMSK5 &= ~(1<<ICIE5);

	if (_radio_status)
	{
		// value
		// Because timer runs at 0.5us we need to do value/2
		pwmValues[0] = constrain((_PWM_RAW[0] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
		pwmValues[1] = constrain((_PWM_RAW[1] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
		pwmValues[2] = constrain((_PWM_RAW[2] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
		pwmValues[3] = constrain((_PWM_RAW[3] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
		pwmValues[4] = constrain((_PWM_RAW[4] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
		pwmValues[5] = constrain((_PWM_RAW[5] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
		pwmValues[6] = constrain((_PWM_RAW[6] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
		pwmValues[7] = constrain((_PWM_RAW[7] >> 1),MIN_PULSEWIDTH,MAX_PULSEWIDTH);
	}

	/* Signal as been read */
    _radio_status = 0;     // Radio channel read
    //	Resume timer 5 interrupt
    TIMSK5 = _timsk5;
}

void PwmApm25::enable_out(uint8_t ch)
{
    switch(ch) {
    case 0: TCCR1A |= (1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A |= (1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A |= (1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A |= (1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A |= (1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A |= (1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A |= (1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A |= (1<<COM3A1); break; // CH_8 : OC3A
    case 9: TCCR5A |= (1<<COM5B1); break; // CH_10 : OC5B
    case 10: TCCR5A |= (1<<COM5C1); break; // CH_11 : OC5C
    }
}

void PwmApm25::disable_out(uint8_t ch)
{
    switch(ch) {
    case 0: TCCR1A &= ~(1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A &= ~(1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A &= ~(1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A &= ~(1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A &= ~(1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A &= ~(1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A &= ~(1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A &= ~(1<<COM3A1); break; // CH_8 : OC3A
    case 9: TCCR5A &= ~(1<<COM5B1); break; // CH_10 : OC5B
    case 10: TCCR5A &= ~(1<<COM5C1); break; // CH_11 : OC5C
    }
}

unsigned char PwmApm25::getState(void)
{
    return(_radio_status);
}

/* ---------------- OUTPUT SPEED CONTROL ------------------ */

void PwmApm25::setFastOutputChannels(uint32_t chmask, uint16_t speed_hz)
{
    uint16_t icr = _map_speed(speed_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2))) != 0) {
        ICR1 = icr;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) | _BV(CH_5))) != 0) {
        ICR4 = icr;
    }

    if ((chmask & ( _BV(CH_6) | _BV(CH_7) | _BV(CH_8))) != 0) {
        ICR3 = icr;
    }
}

} /* namespace hw */

extern "C" ISR(TIMER5_CAPT_vect) {
	hw::PwmApm25::_timer5_capt_cb();
}

#endif
