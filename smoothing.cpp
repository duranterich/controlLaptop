// TMP35 or TMP36 temp sensor connected to Analogue input A1, 3.3volt and ground
// or LM35 temp sensor connected to A1, 5volt and ground
// temp range ~2C to ~105C
// display on serial monitor and/or LCD
// for a TMP36 (-40C to ~55C), change line 45 to:   tempC = total * Aref * 0.1 / numReadings - 50.0;

#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // your LCD pins could be different
byte ledPin = 10; // backlight pin
const byte numReadings = 32; // number of readings for smoothing (max 64)
int readings[numReadings]; // readings from the analog input
byte index = 0; // index of the current reading
unsigned int total = 0; // running total
int inputPin = A1; // the pin that the sensor is connected to
float Aref = 1.0759; // change this value to the actual Aref voltage of ---YOUR--- Arduino (1.0 - 1.2), or adjust to get accurate readings
float tempC; // Celcius
float tempF; // Fahrenheit

void setup() {
  //analogWrite(ledPin, 200); // optional dimming
  analogReference(INTERNAL); // use the internal ~1.1volt reference | change (INTERNAL) to (INTERNAL1V1) for a Mega
  Serial.begin(115200); // ---set serial monitor to this value---
  lcd.begin(16, 2); // shield with 2x16 characters
  lcd.print("Thermometer"); // info text
  lcd.setCursor(0, 1); // second row
  lcd.print("0-100 Celcius");
  for (index = 0; index < numReadings; index++) { // fill the array for faster startup
    readings[index] = analogRead(inputPin);
    total = total + readings[index];
  }
  index = 0; // reset
  delay(2000); // info display time
}

void loop() {
  total = total - readings[index]; // subtract the last reading
  readings[index] = analogRead(inputPin); // one unused reading to clear ghost charge
  readings[index] = analogRead(inputPin); // read from the sensor
  total = total + readings[index]; // add the reading to the total
  index = index + 1; // advance to the next position in the array
  if (index >= numReadings) // if we're at the end of the array
    index = 0; // wrap around to the beginning

  // convert value to temp
  tempC = total * Aref * 0.1 / numReadings; // value to celcius conversion
  tempF = tempC * 1.8 + 32; // Celcius to Fahrenheit conversion

  // print to LCD
  if (total == 1023 * numReadings) { // if overflow
    lcd.clear();
    lcd.print("---TOO HOT---");
  }
  else {
    lcd.clear();
    lcd.print(tempC, 2); // two decimal places
    lcd.setCursor(6, 0); // position 6, first row
    lcd.print("Celcius");
    lcd.setCursor(0, 1); // second row
    lcd.print(tempF, 1); // one decimal place
    lcd.setCursor(6, 1); // position 6, second row
    lcd.print("Fahrenheit");
  }

  // print to serial monitor
  Serial.print("Raw average = ");
  Serial.print(total / numReadings);
  if (total == 1023 * numReadings) {
    Serial.println("  ----too hot----");
  }
  else {
    Serial.print("   The temperature is  ");
    Serial.print(tempC, 2);
    Serial.print(" Celcius  ");
    Serial.print(tempF, 1);
    Serial.println(" Fahrenheit");
  }

  delay(1000); // use a non-blocking delay when combined with other code
}