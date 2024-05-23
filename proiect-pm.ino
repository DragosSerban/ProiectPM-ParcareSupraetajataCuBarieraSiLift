#include <Servo.h> 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PinChangeInterrupt.h>

const int buttonPin1 = 2; // Pin connected to button 1
const int buttonPin2 = 3; // Pin connected to button 2

volatile bool button1Pressed = false;
volatile bool button2Pressed = false;

int irSensorPin1 = 6; // First IR sensor pin
int irSensorPin2 = 7; // Second IR sensor pin

// Define the pin for the servo motor
int servoPin = 5;

int servoPinElevator = 8;

// Create a servo object
Servo myServo;

// Create elevator servo
Servo servoElevator;

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize variables for counting cars
int carCount = 0;
bool carCountUpdated = false;

volatile bool carDetected = false;
unsigned long lastMillis = 0;

// Variable to store the time when the servo was opened
unsigned long servoOpenTime = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  // Set the IR sensor pins as input
  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);
  // Attach the servo to its pin
  myServo.attach(servoPin);
  servoElevator.attach(servoPinElevator);
  servoElevator.write(90);
  myServo.write(0);

  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Cars entered: ");

  attachPCINT(digitalPinToPCINT(irSensorPin1), ir_sensors_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(irSensorPin2), ir_sensors_ISR, CHANGE);

  // Set button pins as input with internal pull-up resistors
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  // Attach interrupts to the button pins
  attachInterrupt(digitalPinToInterrupt(buttonPin1), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), button2ISR, FALLING);
}

// function that opens the barrier
void openServo() {
  myServo.write(90);
}

// function that closes the barrier
void closeServo() {
  myServo.write(0);
}


void ir_sensors_ISR()
{
  if (digitalRead(irSensorPin1) == HIGH && digitalRead(irSensorPin2) == HIGH)
  {
    carCount++;
    Serial.println(carCount);
    
    carDetected = true;
    lastMillis = millis();
  }
  else
  {
    openServo();
    servoOpenTime = millis();
  }
}

// Interrupt Service Routine (ISR) for button 1
void button1ISR() {
  button1Pressed = true;
}

// Interrupt Service Routine (ISR) for button 2
void button2ISR() {
  button2Pressed = true;
}

// Function to move the elevator up
void moveElevatorUp() {
  servoElevator.write(180); // Adjust the angle as needed for your setup
  delay(3000); // Wait for 1 second
  servoElevator.write(90); // Stop elevator
}

// Function to move the elevator down
void moveElevatorDown() {
  servoElevator.write(0); // Adjust the angle as needed for your setup
  delay(3000); // Wait for 1 second
  servoElevator.write(90); // Stop elevator
}

void loop() {
  if (carDetected && millis() - lastMillis >= 1000 && digitalRead(irSensorPin1) == HIGH && digitalRead(irSensorPin2) == HIGH)  // Check if a second has passed
  {
    closeServo();
    carDetected = false;  // Reset the flag
  }

  if (button1Pressed) {
    Serial.println("Button 1 pressed!");
    moveElevatorUp();
    button1Pressed = false; // Reset the flag
  }

  if (button2Pressed) {
    Serial.println("Button 2 pressed!");
    moveElevatorDown();
    button2Pressed = false; // Reset the flag
  }
}
