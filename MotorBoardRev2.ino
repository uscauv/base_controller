//Motor Driver Rev2.0
//USC AUV
//Michael Kukar 2015
//mkukar@gmail.com

//Communicates on serial stream (look at baudRate variable)
//To stop all motors, send
//0
//To send a change to a motor, send
//motor# speedVal
//e.g.
//1 0
//that would set motor 1 to the speed 1500 (which is stop)
//the speedVal is automatically added to 1500 to account for the
//range the motor drivers take input from
//-400 is full reverse, 400 is full forward, 0 is stop

#include <Servo.h>

//THESE NEED TO BE INITIALIZED TO THE PINS THE CONNECT TO
const int m1pin;
const int m2pin;
const int m3pin;
const int m4pin;
const int m5pin;
const int m6pin;

Servo m1;
Servo m2;
Servo m3;
Servo m4;
Servo m5;
Servo m6;

const int baudRate = 9600;
//values for the motor control
const int reverseMax = 1100;
const int stopVal = 1500;
const int forwardMax = 1900;
//values for changing the motor number and speed
int motorIn = 1;
int speedIn = stopVal;

//sets all the motors to the off position
void allOff() {
  Serial.println("Stopping motors...");
  m1.writeMicroseconds(stopVal);
  m2.writeMicroseconds(stopVal);
  m3.writeMicroseconds(stopVal);
  m4.writeMicroseconds(stopVal);
  m5.writeMicroseconds(stopVal);
  m6.writeMicroseconds(stopVal);
  delay(1000); //waits for them to stop (takes 1 second)
  Serial.println("Motors stopped.");
}

void setMotor(int motor, int speedVal) {
  Serial.print("Changing motor #");
  Serial.print(motor);
  Serial.print(" to speed ");
  Serial.print(speedVal);
  Serial.println(".");
  //checks to make sure the speed value is in the acceptable range
  if (speedVal < reverseMax || speedVal > forwardMax) {
    Serial.println("Error: Speed out of bounds.");
    return;
  }
  switch(motor) {
    case 1:
      m1.writeMicroseconds(speedVal);
      break;
    case 2:
      m2.writeMicroseconds(speedVal);
      break;
    case 3:
      m3.writeMicroseconds(speedVal);
      break;
    case 4:
      m4.writeMicroseconds(speedVal);
      break;
    case 5:
      m5.writeMicroseconds(speedVal);
      break;
    case 6:
      m6.writeMicroseconds(speedVal);
      break;
    default:
      Serial.println("Error: Please select motor 1-6.");
      return;
  }
}

void setup() {
  Serial.begin(baudRate); //sets up serial communication
  Serial.println("Initializing motors...");
  m1.attach(m1pin);
  m2.attach(m2pin);
  m3.attach(m3pin);
  m4.attach(m4pin);
  m5.attach(m5pin);
  m6.attach(m6pin);
  delay(100); //takes 1 second to let ESC recognize the motors
  allOff(); //turns all the motors off
  Serial.println("Motors initialized.");
}

void loop() {
  //waits for a serial value to change the motors
  if (Serial.available() > 0) {
    //takes in input
    motorIn = Serial.parseInt();
    speedIn = Serial.parseInt();
    speedIn = speedIn + 1500; //adds 1500 to the speed to center it (0 is stop)
    //prints out what it recieved
    Serial.println("Recieved data.");
    if (motorIn == 0) { //case from computer to stop all motors
      allOff();
    }
    else {
      setMotor(motorIn, speedIn);
    }
  }
  delay(1); //waits in between checking to not use as much power
}
