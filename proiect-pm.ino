#include <Servo.h> 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PinChangeInterrupt.h>
#include <NewPing.h>
#include <TimerOne.h>

const int buttonPin1 = 2; // pin connected to button 1
const int buttonPin2 = 3; // pin connected to button 2

volatile bool button1Pressed = false;
volatile bool button2Pressed = false;

int ultrasonicEchoPin1 = 6; // first ultrasonic sensor echo pin
int ultrasonicTrigPin1 = 7; // first ultrasonic sensor trig pin

int ultrasonicEchoPin2 = 9; // second ultrasonic sensor echo pin
int ultrasonicTrigPin2 = 10; // second ultrasonic sensor trig pin

int servoBarrierPin = 5; // pin for the servomotor used as barrier
int servoElevatorPin = 8; // pin for the servomotor used as elevator

// create a servo object for the barrier
Servo servoBarrier;
// create a servo object for the elevator
Servo servoElevator;

// set the LCD address to 0x27 (we have a 16 chars and 2 line display)
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool carDetectedNearBarrier = false;

volatile unsigned int distance = 0;
volatile bool newPing = false;

void setup() {
  // initialize the serial communication
  Serial.begin(9600);

  // set button pins as input with internal pull-up resistors
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  // attach interrupts to the button pins
  attachInterrupt(digitalPinToInterrupt(buttonPin1), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), button2ISR, FALLING);

  // set the barrier servo angle to 0 degrees
  servoBarrier.attach(servoBarrierPin);
  servoBarrier.write(0);

  // set the elevator servo to a still position
  servoElevator.attach(servoElevatorPin);
  servoElevator.write(90);

  // initialize the LCD
  lcd.begin();

  // turn on the blacklight and print current number of cars which entered parking space
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Number of cars:");
  lcd.setCursor(1, 1);
  lcd.print("0");

  // set ultrasonic sensor pins
  pinMode(ultrasonicTrigPin1, OUTPUT);
  pinMode(ultrasonicEchoPin1, INPUT);
  pinMode(ultrasonicTrigPin2, OUTPUT);
  pinMode(ultrasonicEchoPin2, INPUT);
}

// Interrupt Service Routine (ISR) for button 1
void button1ISR() {
  button1Pressed = true;
}

// Interrupt Service Routine (ISR) for button 2
void button2ISR() {
  button2Pressed = true;
}

// function used for measuring distance using an ultrasonic sensor
unsigned int measureDistance(int trigPin, int echoPin) {
  // Send a 10us pulse to trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure the duration of the echo pulse
  unsigned long duration = pulseIn(echoPin, HIGH);

  // calculate the distance in centimeters and then return it
  unsigned int distance = duration * 0.034 / 2;

  return distance;
}

// function that opens the barrier
void openBarrier() {
  servoBarrier.write(90);
}

// function that closes the barrier
void closeBarrier() {
  servoBarrier.write(0);
}

// function used for moving the elevator up
void moveElevatorUp() {
  servoElevator.write(180); // start the elevator
  delay(3000); // move it for 3 seconds
  servoElevator.write(90); // stop the elevator
}

// function used for moving the elevator down
void moveElevatorDown() {
  servoElevator.write(0); // start the elevator
  delay(3000); // move it for 3 seconds
  servoElevator.write(90); // stop the elevator
}

void loop() {
  // measure the distance using the first ultrasonic sensor
  unsigned int distance1 = measureDistance(ultrasonicTrigPin1, ultrasonicEchoPin1);
  // measure the distance using the second ultrasonic sensor
  unsigned int distance2 = measureDistance(ultrasonicTrigPin2, ultrasonicEchoPin2);

  // check if there is any car in proximity to the barrier
  if (distance1 < 10 || distance2 < 10)
  {
    openBarrier();
    carDetectedNearBarrier = true;
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Car detected: ");
    lcd.setCursor(1, 1);
    lcd.print(min(distance1, distance2));
  }
  else if (carDetectedNearBarrier)
  {
    delay(1000);
    closeBarrier();
    carDetectedNearBarrier = false;
  }

  delay(100);

  if (button1Pressed)
  {
    button1Pressed = false;
    moveElevatorUp();
  }
}
