// Overblown Fan Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const byte fanPWM = 9;			    // fan control connected to digital pin 3
const byte fanRPMPin = 0;				// fan RPM readings

int dutyCycleStart = 100;              // startup duty cycle (0-255)
int dutyCycleFan = 150;

int fanRPM;								// variable to output fan RPM
int rpmCounter;						// counter for number of interrupts (hall effect) per second

int32_t pwmFrequency = 25000;                                  // set PWM frequency for system to 27.5kHz

void setup() {
	InitTimersSafe();					// initialize all timers except for 0, to save time keeping functions
	Serial.begin(115200);       
	delay(100);

	// RPM Counter Setup
	pinMode(2, INPUT_PULLUP);
	attachInterrupt(fanRPMPin, rpm, RISING);
           
	// 120mm Radiator Fans Setup
  SetPinFrequencySafe(fanPWM,pwmFrequency);
	pinMode(fanPWM, OUTPUT);
	pwmWrite(fanPWM,dutyCycleStart);

}

// Interrupt to count the fan RPM

void loop() {
  
    pwmWrite(fanPWM,dutyCycleFan);
    
    rpmCounter = 0;             // reset rotation counter to 0
    sei();                  // enable interrupts
    delay(1000);              // pause 1 second
    cli();                  // disable interrupts
  
    fanRPM = (rpmCounter * 60) / 2;     // take count of interrupts over 1 second, multiply by 60 (for minute) and div by 2 for bipolar hall effect sensor
    Serial.print("Fan RPM: ");        // output fan RPM
    Serial.println(fanRPM);

    delay(1000);
    
}

void rpm() {
  rpmCounter++;
}
