// Overblown Fan Control
// by: Richard Durante

#include <PWM.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TempPIN 0                           // Analog Pin 0
#define TempPINAmb 1                        // Analog Pin 1

float ardVolt = 4.95;                       // Arduino Output Voltage 5V measured
                                        
int temperatureSense;                       // Temperature of sensor
int temperatureAmb;                         // Ambient Temperature Sensor     

// Fan Pins
const byte fanPWM = 9;			                // fan control connected to digital pin 3
const byte fanRPMPin = 2;				            // fan RPM readings

// Fan Duty Cycle
int dutyCycleStart = 50;                    // startup duty cycle (0-255)
int dutyCycleFan;

// For Looping Temperature Readings
const byte numReadings = 50;                // number of readings to average
int rawVal[numReadings];                    // setup array for calculating the raw value
byte index = 0;                             // index for for loop
int total = 0;                              // running total

// For Looping Temperature Readings
const byte numReadings2 = 50;               // number of readings to average
int rawVal2[numReadings2];                  // setup array for calculating the raw value
byte index2 = 0;                            // index for for loop
int total2 = 0;                             // running total

int fanRPMVal;								                  // variable to output fan RPM
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
  pinMode(TempPINAmb, INPUT);
        
	// 120mm Radiator Fans Setup
  SetPinFrequencySafe(fanPWM,pwmFrequency);
	pinMode(fanPWM, OUTPUT);
	pwmWrite(fanPWM,dutyCycleStart);
  delay(10000);

  Serial.print("TemperatureSurface(C), TemperatureAmb(C), FanRPM, FanDuty(%)");
  Serial.println("");
  // init temperatureSense
  // rawVal = analogRead(TempPIN);
  // temperatureSense = tempSensor(rawVal);
  // prevTemp = temperatureSense;

  for (index = 0; index < numReadings; index++) {     // fill the array for faster startup
    rawVal[index] = analogRead(TempPIN);
    total = total + rawVal[index];
  }
  index = 0;                                          // reset
  
  for (index2 = 0; index2 < numReadings2; index2++) { // fill the array for faster startup
    rawVal2[index2] = analogRead(TempPINAmb);
    total2 = total2 + rawVal2[index2];
  }
  index2 = 0;                                         // reset
  
}

// Interrupt to count the fan RPM
 void rpm() {
  rpmCounter++;
}

void loop() {
    total = total - rawVal[index];          // subtract the last reading
    rawVal[index] = analogRead(TempPIN);    // one unused reading to clear ghost charge
    rawVal[index] = analogRead(TempPIN);    // read from the sensor
    total = total + rawVal[index];          // add the reading to the total
    index = index + 1;                      // advance to the next position in the array
    if (index >= numReadings){              // if we're at the end of the array
      index = 0;                            // wrap around to the beginning
    }

    temperatureSense = tempSensor((total/numReadings));      // read ADC and  convert it to Celsius
    
    total2 = total2 - rawVal2[index2];          // subtract the last reading
    rawVal2[index2] = analogRead(TempPINAmb);    // one unused reading to clear ghost charge
    rawVal2[index2] = analogRead(TempPINAmb);    // read from the sensor
    total2 = total2 + rawVal2[index2];          // add the reading to the total
    index2 = index2 + 1;                      // advance to the next position in the array
    if (index2 >= numReadings2){              // if we're at the end of the array
      index2 = 0;                            // wrap around to the beginning
    }

    temperatureAmb = tempSensor((total2/numReadings2));      // read ADC and  convert it to Celsius
    
    // Control for PWM
    dutyCycleFan = 75.369 * exp(0.0305 * temperatureSense);    // fan curve based on excel spreadsheet
     
    if(dutyCycleFan > 255){                                 // if temperatures exceed actuals, error check
      dutyCycleFan = 255;                                   // this should be corrected from previous error check
    }
    else if(dutyCycleFan < 0){
      dutyCycleFan = 0;
    }

     pwmWrite(fanPWM,dutyCycleFan);                          // write duty cycle to fan
     fanRPMVal = fanRPMOutput(rpmCounter);    

    //Serial.print("Temperature(Celsius): ");
    Serial.print(temperatureSense);                       // display Celsius
    Serial.print(",");                                    // delimeter
    Serial.print(temperatureAmb);                         // display Ambient
    Serial.print(",");                                    // delimeter
    Serial.print(fanRPMVal);                              // display RPMs
    Serial.print(",");                                    // delimter
    Serial.print(dutyCycleFan);                           // display Duty Cycle
    Serial.println("");
    
    delay(1000);                                          // data logging interval
    
}

int fanRPMOutput(int freqFan) {
	int fanRPM;

	freqFan = 0;							// reset rotation counter to 0
	sei();									// enable interrupts
	delay(1000);							// pause 1 second
	cli();									// disable interrupts

	fanRPM = (freqFan * 60) / 2;			// take count of interrupts over 1 second, multiply by 60 (for minute) and div by 2 for bipolar hall effect sensor
	Serial.print("Fan RPM: ");				// output fan RPM
	Serial.println(fanRPM, DEC);
  return fanRPM;
}

float tempSensor(float RawADC) {  
  float TempSurface;                                  // Dual-Purpose variable to save space.
  RawADC = RawADC * ardVolt;
  RawADC /= 1024.0;
  TempSurface = (RawADC - 0.5) * 100 ;          //converting from 10 mv per degree wit 500 mV offset
  
  return TempSurface;
}
