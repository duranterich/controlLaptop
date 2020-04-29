// Overblown Fan Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TempPIN 0                           // Analog Pin 0

float ardVolt = 4.95;                       // Arduino Output Voltage 5V measured
                                        
float temperatureSense;                     // Temperature of sensor     

// Fan Pins
const byte fanPWM = 9;			                // fan control connected to digital pin 3
const byte fanRPMPin = 2;				            // fan RPM readings

// Fan Duty Cycle
int dutyCycleStart = 100;                   // startup duty cycle (0-255)
int dutyCycleFan;

// For Looping Temperature Readings
const byte numReadings = 50;                // number of readings to average
float rawVal[numReadings];                    // setup array for calculating the raw value
byte index = 0;                             // index for for loop
float total = 0;                     // running total

// int fanRPM;								              // variable to output fan RPM
int rpmCounter;						                  // counter for number of interrupts (hall effect) per second

int32_t pwmFrequency = 24000;               // set PWM frequency for system to 27.5kHz

void setup() {
	InitTimersSafe();					                // initialize all timers except for 0, to save time keeping functions
	Serial.begin(115200);       
	delay(100);

	// RPM Counter Setup
	pinMode(fanRPMPin, INPUT);
	attachInterrupt(fanRPMPin, rpm, RISING);
        
  // Temperature Sensor Setup
  pinMode(TempPIN, INPUT);
        
	// 120mm Radiator Fans Setup
  SetPinFrequencySafe(fanPWM,pwmFrequency);
	pinMode(fanPWM, OUTPUT);
	pwmWrite(fanPWM,dutyCycleStart);
  delay(5000);

  // init temperatureSense
  // rawVal = analogRead(TempPIN);
  // temperatureSense = tempSensor(rawVal);
  // prevTemp = temperatureSense;

  for (index = 0; index < numReadings; index++) { // fill the array for faster startup
    rawVal[index] = analogRead(TempPIN);
    total = total + rawVal[index];
  }
  index = 0; // reset
  
}

// Interrupt to count the fan RPM
 void rpm() {
  rpmCounter++;
}

void loop() {
    total = total - rawVal[index]; // subtract the last reading
    rawVal[index] = analogRead(TempPIN); // one unused reading to clear ghost charge
    rawVal[index] = analogRead(TempPIN); // read from the sensor
    total = total + rawVal[index]; // add the reading to the total
    index = index + 1; // advance to the next position in the array
    if (index >= numReadings){ // if we're at the end of the array
      index = 0; // wrap around to the beginning
    }
    // rawVal = analogRead(TempPIN);
    temperatureSense = tempSensor((total/numReadings));      // read ADC and  convert it to Celsius

   // if(temperatureSense > 36){                  // temperature error correction, possibly due to no diode
   //     temperatureSense = prevTemp;
   // }
   // else if(temperatureSense < 18){
   //     temperatureSense = prevTemp;
   // }
   // prevTemp = temperatureSense;

    // Control for PWM
    dutyCycleFan = 22.554 * exp(0.0665 * temperatureSense);    // fan curve based on excel spreadsheet
     
    if(dutyCycleFan > 255){                                 // if temperatures exceed actuals, error check
      dutyCycleFan = 255;                                   // this should be corrected from previous error check
    }
    else if(dutyCycleFan < 50){
      dutyCycleFan = 50;
    }

     pwmWrite(fanPWM,dutyCycleFan);                          // write duty cycle to fan
    

    Serial.print("Temperature(Celsius): ");
    Serial.print(temperatureSense,2);                       // display Celsius
    Serial.println("");
    
    //Serial.print("Analog Val : ");
    //Serial.print(rawVal);                                 // display mV
    //Serial.println(""); 

    Serial.print("Fan Duty: ");
    Serial.print(dutyCycleFan);                           // display duty cycle
    Serial.println("");
    // fanRPMOutput(rpmCounter);

    delay(1000);
    
}

int fanRPMOutput(int rpmCounter) {
	int fanRPM;

	rpmCounter = 0;							// reset rotation counter to 0
	sei();									// enable interrupts
	delay(1000);							// pause 1 second
	cli();									// disable interrupts

	fanRPM = (rpmCounter * 60) / 2;			// take count of interrupts over 1 second, multiply by 60 (for minute) and div by 2 for bipolar hall effect sensor
	Serial.print("Fan RPM: ");				// output fan RPM
	Serial.println(fanRPM, DEC);

}

float tempSensor(float RawADC) {  
  float TempSurface;                                  // Dual-Purpose variable to save space.
  RawADC = RawADC * ardVolt;
  RawADC /= 1024.0;
  TempSurface = (RawADC - 0.5) * 100 ;          //converting from 10 mv per degree wit 500 mV offset
  
  return TempSurface;
}
