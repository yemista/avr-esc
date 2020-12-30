#ifndef F_CPU
#define F_CPU 8000000UL 
#endif
 
#include <avr/io.h>
#include <util/delay.h>  

// pins each FET is connected to
#define     HBRIDGE1_START          0x1
#define     HBRIDGE1_END            0x2
#define     HBRIDGE2_START          0x4
#define     HBRIDGE2_END            0x8

// pin that the input PWM signal comes from
#define     PWM_INPUT               0x10              
 
#define     TRUE                    1
#define     FALSE                   0


// how to interpret the PWMm input signal
#define     PWM_FOWARD_MIN          1
#define     PWM_REVERSE_MIN         -1


#define     TRANSLATE_PWM(S)        (S / 100)


// motor states
#define     breakMotor                   0x0
#define     FORWARD                 0x1
#define     REVERSE                 0x2



void initializeIO() {
    DDRB = HBRIDGE1_START | HBRIDGE1_END | HBRIDGE2_START | HBRIDGE2_END;
    _delay_ms(1);
}

char readInputSignal() {
    return PINB & PWM_INPUT;
}

void breakMotor() {
    enableHBridge1Timer(FALSE);
    enableHBridge2Timer(FALSE);
    PORTB = HBRIDGE1_END | HBRIDGE2_END;
}

void goForward(char pwmVal) {
    enableHBridge1Timer(TRUE);
}

void updateForward(char pwmVal, char currentState) {
    if (currentState == REVERSE) {
        breakMotor();
        _delay_ms(10);
    }

    goForward(pwmVal);
}

void updateReverse(char pwmVal, char currentState) {
    if (currentState == FORWARD) {
        breakMotor();
        _delay_ms(10);
    }

    goReverse(pwmVal);
}

char updateHBridge(char pwmVal, char currentState) {
    if (pwmVal > PWM_FOWARD_MIN) {
        updateForward(pwmVal, currentState);
        return FORWARD;
    } else if (pwmVal < PWM_REVERSE_MIN) {
        updateReverse(pwmVal, currentState);
        return REVERSE;
    } else {
        breakMotor();
        return breakMotor;
    }
}

int main(void) {
    initializeIO();

    char inputSignalPrev = 0;
    char inputSignal = inputSignalPrev;
    char currentState = breakMotor;

    while (TRUE) {
        inputSignal = readInputSignal();
        
        if (inputSignal != inputSignalPrev) {
            currentState = updateHBridge(inputSignal, currentState);
            inputSignalPrev = inputSignal;        
        }        
    }
}