#include <Wire.h>
#include "rgb_lcd.h"
#include <Servo.h>  // Include Servo library

// Pin definitions
#define anInput A0      // MQ135 sensor analog pin
#define buzzerPin 8     // Buzzer connected to pin 8
#define redLEDPin 2     // Red LED connected to pin 2
#define blueLEDPin 3    // Blue LED connected to pin 3
#define servoPin 9      // Servo connected to pin 9
#define co2Zero 55      // Calibration value for CO2 (example)
#define threshold 1000  // CO2 threshold to trigger buzzer and servo

rgb_lcd lcd;  // LCD object
Servo myServo;  // Servo object

void setup() {
  pinMode(anInput, INPUT);
  pinMode(buzzerPin, OUTPUT);  // Buzzer pin as output
  pinMode(redLEDPin, OUTPUT);  // Red LED pin
  pinMode(blueLEDPin, OUTPUT); // Blue LED pin
  
  Serial.begin(9600);

  lcd.begin(16, 2);        // Initialize the LCD (16x2)
  lcd.setRGB(0, 0, 255);   // Set background color to blue
  lcd.print("CO2 Monitor");
  delay(2000);
  lcd.clear();

  myServo.attach(servoPin);  // Attach the servo to pin 9
  myServo.write(90);         // Initialize the servo to 90 degrees (normal position)
  delay(2000);               // Wait for 2 seconds
}

void loop() {
  int co2now[10];
  int co2raw = 0;
  int co2ppm = 0;
  int co2Sum = 0;

  // Take 10 readings to average the CO2 level
  for (int x = 0; x < 10; x++) {
    co2now[x] = analogRead(anInput);
    delay(200);
  }

  // Average the readings
  for (int x = 0; x < 10; x++) {
    co2Sum += co2now[x];
  }

  co2raw = co2Sum / 10;
  co2ppm = co2raw - co2Zero;

  // Print raw sensor value and calculated PPM to the Serial Monitor for debugging
  Serial.print("Raw CO2 sensor value: ");
  Serial.println(co2raw);
  Serial.print("CO2 PPM (after calibration): ");
  Serial.println(co2ppm);

  // Display CO2 reading on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);  // First line
  lcd.print("CO2: ");
  lcd.print(co2ppm);
  lcd.print(" PPM");

  // If CO2 exceeds the threshold, turn on the buzzer, red LED and move the servo to 150 degrees
  if (co2ppm >= threshold) {
    digitalWrite(buzzerPin, HIGH);  // Turn on buzzer
    digitalWrite(redLEDPin, HIGH);  // Turn on red LED
    digitalWrite(blueLEDPin, LOW);  // Turn off blue LED
    myServo.write(150);             // Move the servo to 150 degrees
    lcd.setCursor(0, 1);
    lcd.print("CO2 High!");
  } else {
    digitalWrite(buzzerPin, LOW);   // Turn off buzzer
    digitalWrite(redLEDPin, LOW);   // Turn off red LED
    digitalWrite(blueLEDPin, HIGH); // Turn on blue LED
    myServo.write(90);              // Move the servo to 90 degrees (normal position)
    lcd.setCursor(0, 1);
    lcd.print("CO2 Normal");
  }

  delay(1000);  // Delay before the next reading
}
