#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "DFR_Key.h"

DFR_Key keypad;         // define keypad object
Servo myservo;                                // define a Servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define a BNO055 object


LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);         // config the lcd using these pins (don't change the numbers)

//////////// Global variables that may change across the functions /////////////
int STEERANGLE = 90;                          // servo initial angle (range is 0:180 and 90 is middle)
float HEADING = 0;                            // actual heading varaiable
int carSpeedPin = 2;                          // define a pin for DC motor (it should support PWM)
int carSpeed = 20;                            // define a variable for DC motor speed
float errorHeadingRef = 0;                    // define a variable for the error between actual heading and reference
int localkey = 0;                             // variable for reading the keypad value
int ref = 0;                                  // reference angle

// Our Vars
imu::Vector<3> euler;                         // Vector of IMU
int distL = 0;                                // The distance left in degrees to desired heading
int distR = 0;                                // The distance right in degrees to desired heading
int steeringAngle = 0;                        // angle to turn the wheel to get to desired heading
int selectedTime = 0;

void setup() {
  myservo.attach(44);                         // set which pin the servo is connected to (here pin 44)
  lcd.begin( 16, 2 );                         // LCD type (col , row) (it's 16 x 2)
  Serial.begin(9600);                         // setup the serial for monitoring (baudrate = 9600)
  Serial.println("Orientation Sensor Calibration"); Serial.println("");

  ///reading the reference///
  localkey = 0;
  while (localkey != 1) {
    lcd.clear();
    localkey = keypad.getKey();
    if (localkey == 3)
      ref = ref + 30;
    if (localkey == 4)
      ref = ref - 30;
    if (ref < 0)
      ref = 0;
    if (ref > 360)
      ref = 360;
    lcd.print(ref);
    delay(100);
  }
  selectedTime = millis();

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");   // check if the sensor is connected and deteced
    while (1);
  }

  byte c_data[22] = {0, 0, 0, 0, 0, 0, 92, 254, 115, 3, 39, 1, 1, 0, 254, 255, 1, 0, 232, 3, 58, 5};    ///////////////// PASTE YOUR CALIBRATION DATA HERE /////////////
  //byte c_data[22] = {0, 0, 0, 0, 0, 0, 226, 253, 99, 3, 20, 2, 1, 0, 254, 255, 1, 0, 232, 3, 212, 3};
  bno.setCalibData(c_data);                                                                                       // Save calibration data
  delay(1000);
  bno.setExtCrystalUse(true);

  analogWrite(carSpeedPin, carSpeed);         // set the pwm duty cycle

  FlexiTimer2::set(100, navigate);            // define a timer interrupt with a period of "100*0.001 = 0.1" s or 10 Hz
  FlexiTimer2::start();                       // start the timer interrupt
}



void ReadHeading() { // Input: Nothing - // Output: HEADING
  // Read the heading using VECTOR_EULER from the IMU
  // that is more stable than any VECTOR_MAGNETOMETER.
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}



void CalculateSteering() { // Input: HEADING & Reference Heading - // Output: Steering Angle
  // Calculate the steering angle according to the referece heading and actual heading

  float x = euler.x() + 10.37;

  if (x > 360)
  {
    x -= 360;
  }

  // calculate the distance left and right to desired heading
  ///////////////////////////////////////////////////////////
  if(x > ref) {

    distL = x - ref;
    distR = (360 - x) + ref;
    
  } else {

    distR = ref - x;
    distL = (360 - ref) + x;
    
  }
  ///////////////////////////////////////////////////////////

  // Set the steering angle
  ///////////////////////////////////////////////////////////
  if (distL < distR) { 
    steeringAngle = 60; // turn left all the way, it's closer...
    
    if(distL < 45)
     steeringAngle = 70;

    if(distL < 30)
     steeringAngle = 80;

    if(distL < 15)
     steeringAngle = 85;
    
  } else {
    steeringAngle = 125; // turn right, it's closer...

    if(distR < 45)
     steeringAngle = 115;

    if(distR < 30)
     steeringAngle = 105;

    if(distR < 15)
     steeringAngle = 100;
    
  }

  // x == ref angle, +/- 1
  //if(x >= ref - 2 && x <= ref + 2) {
  if(x == ref) {
    steeringAngle = 93; // go straight
  }
  ///////////////////////////////////////////////////////////
  
}

void Actuate() { // Input: Steering angle - Output: nothing
  if ((millis() - selectedTime) > 30000) { // after 30 seconds (from the ref selected)
    // Stop the vehicle (set the speed = 0)
    carSpeed = 0;
    analogWrite(carSpeedPin, carSpeed);
    
  } else {

    // set the steering angle
    myservo.write(steeringAngle);
    
    // set the speed
    carSpeed = 20;
    analogWrite(carSpeedPin, carSpeed);
  }
}

void navigate() {         // This function will be called every 0.1 seconds (10Hz)
  sei();                  // set the interrupt flag ** THIS IS VERY IMPORTANT **. You need to set the interrupt flag or the programm will get stuck here!!!
  ReadHeading();          // read the heading angle
  CalculateSteering();    // calculate the desired steering angle
  Actuate();              // actuate (set the steering angle)
}

void printHeadingOnLCD() {
  // print the heading data on serial monitor to verify the actual heading
  lcd.setCursor(0,0);
  lcd.print("H: ");
  
  if ((euler.x() + 10.37) > 360)
  {
    lcd.print((euler.x() + 10.37) - 360);
  }
  else
  {
    lcd.print(euler.x() + 10.37);
  }
  
  //lcd.print(euler.x() + 10.37);
}

void printSteerAngleOnLCD() {
  // print the steering angle to verify your control command
  lcd.setCursor(0,1);
  lcd.print("SA: ");
  lcd.print(steeringAngle);
  
}

void loop()
{
  lcd.clear();            // clear the LCD
  printHeadingOnLCD();
  printSteerAngleOnLCD();
  delay(100);
}
