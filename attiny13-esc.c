#ifndef F_CPU
#define F_CPU 9600000UL 
#endif
 
#include <avr/io.h>
#include <util/delay.h>  
#include <avr/interrupt.h>



// pins each FET is connected to
#define     HBRIDGE1_START          (1 << PB1)
#define     HBRIDGE1_END            (1 << PB2)
#define     HBRIDGE2_START          (1 << PB3)
#define     HBRIDGE2_END            (1 << PB4)

// pin that the input PWM signal comes from
#define     PWM_INPUT               (1 << PB0)              
 
#define     TRUE                    1
#define     FALSE                   0


// how to interpret the RC input signal
#define     RC_FOWARD_MIN          1600
#define     RC_REVERSE_MIN         1400
#define     RC_SIGNAL_OFF          0

// min and max RC input signals
#define     RC_MAX_VALUE           2000
#define     RC_MIN_VALUE           1000


// max PWM value to check against in timer1 interrupr
#define     MAX_PWM_VALUE          250  

// how often to trigger compare match A interrupt
#define     COMPARE_MATCH_A        0xff

// function for translating input PWM to motor sp(eed PWM
#define     TRANSLATE_FORWARD_PWM(S)        ((S - 1500) / 2)
#define     TRANSLATE_REVERSE_PWM(S)        ((1500 - S) / 2)


// motor states
#define     BREAK                   0x1
#define     FORWARD                 0x2
#define     REVERSE                 0x4
#define     MOTORS_OFF              0



//globals
volatile uint8_t motor_direction = BREAK;

// timer0 variables
volatile uint8_t timer0_ovf = 0;
volatile uint8_t count = 0;
volatile uint16_t pulses = 0;

// timer1 variables
volatile uint8_t motors_on = FALSE;
volatile uint16_t timer1_compa_matches = 0;
volatile uint16_t pwm_threshold = 0;

// watchdog timer variables
volatile uint8_t pin_changed_int_fired = FALSE;

// scalar for timer overflows -> ms
uint16_t timer_scalar = 0;

// inline methods for controlling h-bridge direction
static inline void enableForwardGates() {
    PORTB = HBRIDGE1_START | HBRIDGE1_END;
}

static inline void enableReverseGates() {
    PORTB = HBRIDGE2_START | HBRIDGE2_END;
}

static inline void enableBreakGates() {
    PORTB = HBRIDGE1_END | HBRIDGE2_END;
}

static inline void disableAllGates() {
    PORTB = 0;
}


// **********************************************
// ***** ISR - interrupt service routine(s) *****
// **********************************************

// called when an interrupt enabled pin changes its state
ISR(PCINT0_vect){
    pin_changed_int_fired = TRUE;

    if (PINB & PWM_INPUT) {
        timer1_compa_matches = 0;
        timer0_ovf = 0;   // reset timer overflow counter at rising edge pin change interrupt
        TCNT0 = 0;   // initialize counter register at rising edge pin change interrupt
    } else {
        pulses = timer0_ovf * timer_scalar;
    }
}

ISR(TIM0_OVF_vect) { 
    timer0_ovf++;   // count timer overflows since reset in rising edge pin change interrupt
}

ISR(TIM0_COMPA_vect) {
    timer1_compa_matches++;

    if (timer1_compa_matches > MAX_PWM_VALUE) {
        timer1_compa_matches = 0;
    }
   
    if (timer1_compa_matches < pwm_threshold) {
        if (motors_on) {
            return;
        }

        motors_on = TRUE;

        if (motor_direction & FORWARD) {
            enableForwardGates();
        } else if (motor_direction & REVERSE) {
            enableReverseGates();
        } else if (motor_direction & BREAK) {
            enableBreakGates();
        } else {
            disableAllGates();
        }
    } else {
        // if (motor_direction & BREAK) {
        //     return;
        // }

        if (!motors_on) {
            return;
        }

        motors_on = FALSE;
        disableAllGates();
    }
}

ISR(WDT_vect) {
    if (pin_changed_int_fired) {
        pin_changed_int_fired = FALSE;
    } else {
        pulses = 0;
    }

    // re-enable the watchdog interrupt
    WDTCR |= (1 << WDTIE);
}



//initialization  methods
static inline void enableInterrupts() {
    GIMSK |= (1 << PCIE);   // generally enable pin change interrupt
    PCMSK |= (1 << PCINT0);   // enable pin change interrupt on PB0 (pin 5)
    TIMSK0 |= (1 << TOIE0) ;   // Timer/Counter0 Overflow Interrupt Enable
    TIMSK0 |= (1 << OCIE0A);    // Enable compare match A interrupts
}

static inline void enableOutputs() {
    DDRB = HBRIDGE1_START | HBRIDGE1_END | HBRIDGE2_START | HBRIDGE2_END;
    _delay_ms(10);
}

// this timer is used to measure RC pulse lengths
static inline void setupTimer0() {
    GTCCR |= (1 << TSM) | (1 << PSR10);   // halt timer
    TCCR0B |= (1 << CS00);   // set prescaler to 1
    TCNT0 = 0;   // initialize timer to 0
    OCR0A = COMPARE_MATCH_A;   // initialize compare match A value
    TCCR0A |= (1 << WGM01);
    MCUCR |= (1 << ISC00);
    GTCCR &= ~(1 << TSM);   // start timer
}

static inline void setupWatchdogTimerAsInterrupt() {
    MCUSR = 0;          // clear MCU Status Register
    WDTCR = 0;

    // Setup Watchdog for interrupt and not reset, approximately 500ms timeout
    WDTCR = (1 << WDTIE) | (1 << WDP2) | (1 << WDP0);
}

/**
    Ftimer = CPU Frequency / Prescalar

    Ftimer = 9.6MHz / 1 = 9.6MHz

    tick = 1 / 9.6MHz = 0.000000104s

    total = 0.000000104s * 255 = 0.00002652s

    0.00002652s = 0.02652ms per tick

    1 tick = .02652 ms
    
    1ms = 1 tick / 0.02652ms per tick = 37.7 ticks ~ 38 ticks

    timer_scalar = 1000 / numTicks per ms = 1000 / 38 = 26
*/
static inline void setTimerScalar() {
    timer_scalar = 26;
}

static inline void initialize() {
    cli();
    enableInterrupts();
    setupTimer0();
    setupWatchdogTimerAsInterrupt();
    enableOutputs();
    setTimerScalar();
    sei();
}

static void updateForward(uint16_t pwmVal) {
    if (motor_direction == REVERSE) {
        motor_direction = BREAK;
        _delay_ms(10);
    }

    pwmVal = pwmVal > RC_MAX_VALUE ? RC_MAX_VALUE : pwmVal;
    pwm_threshold = TRANSLATE_FORWARD_PWM(pwmVal);
    motor_direction = FORWARD;
}

static void updateReverse(uint16_t pwmVal) {
    if (motor_direction == FORWARD) {
        motor_direction = BREAK;
        _delay_ms(10);
    }

    pwmVal = pwmVal < RC_MIN_VALUE ? RC_MIN_VALUE : pwmVal;
    pwm_threshold = TRANSLATE_REVERSE_PWM(pwmVal);
    motor_direction = REVERSE;
}

static void disableHBridge() {
    motors_on = FALSE;
    motor_direction = MOTORS_OFF;
    disableAllGates();
}

static void updateHBridge(uint16_t rcInput) {
    if (rcInput == RC_SIGNAL_OFF) {
        disableHBridge();
    } else if (rcInput >= RC_FOWARD_MIN) {
        updateForward(rcInput);
    } else if (rcInput <= RC_REVERSE_MIN) {
        updateReverse(rcInput);
    } else {
        motor_direction = BREAK;
    }
}

int main(void) {
    initialize();
    _delay_ms(100);

    uint16_t inputSignalPrev = 0;
    uint16_t inputSignal = inputSignalPrev;

    // wait until we actually read a signal so we dont start incorrectly
    while (inputSignal == inputSignalPrev) {
        inputSignal = pulses;
    }

    while (TRUE) {
        inputSignal = pulses;
        
        if (inputSignal != inputSignalPrev) {
            updateHBridge(inputSignal);
            inputSignalPrev = inputSignal;      
        }       
    }
}