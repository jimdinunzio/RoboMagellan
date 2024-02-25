/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <Servo.h>
#include <MsTimer2.h>

// Assign your channel out pins
#define THROTTLE_OUT_PIN 5
#define STEERING_OUT_PIN 6

#define NEUTRAL_PWM 1500
#define FULL_FORWARD_PWM 2000
#define FULL_REVERSE_PWM 1000
#define FULL_RIGHT_PWM 2000
#define FULL_LEFT_PWM 1000

// Servo objects 
Servo servoThrottle;
Servo servoSteering;

// channels have new signals flags
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// temporary variables
static uint16_t newThrottle;
static uint16_t newSteering;
static uint8_t bUpdateFlags;

int incomingByte = 0; // for incoming serial data

// blink
boolean output = HIGH;

void flash();

void setup()
{
  Serial.begin(115200);
  Serial.println("RC Signal Test");

  // attach servo objects  
  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  MsTimer2::set(500, flash);
  MsTimer2::start();
  bUpdateFlags = THROTTLE_FLAG | STEERING_FLAG;
  newThrottle = NEUTRAL_PWM;
  newSteering = NEUTRAL_PWM;
}

uint16_t getSteeringByPercent(int8_t percent)
{
    return (percent * (int) ((FULL_RIGHT_PWM - FULL_LEFT_PWM) / 2) + NEUTRAL_PWM) / 100;
}

uint16_t getThrottleByPercent(int8_t percent)
{
  return (percent * (int) ((FULL_FORWARD_PWM - FULL_REVERSE_PWM) / 2) + NEUTRAL_PWM) / 100;
}

void loop()
{
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    //Serial.println(incomingByte, DEC);

    switch (incomingByte) {
      case 'r':
        newSteering = FULL_RIGHT_PWM;
        bUpdateFlags |= STEERING_FLAG;
        Serial.println("Turning right.");
        break;
      case 'l':
        newSteering = FULL_LEFT_PWM;
        bUpdateFlags |= STEERING_FLAG;
        Serial.println("Turning left.");
        break;
      case 'f':
        newThrottle = getThrottleByPercent(10);
        bUpdateFlags |= THROTTLE_FLAG;
        Serial.println("Go forward 10\% power.");
        break;
      case 'b':
        newThrottle = getThrottleByPercent(-10);
        bUpdateFlags |= THROTTLE_FLAG;
        Serial.println("Go backwards 10\% power.");
        break;
      case 'n':
        newThrottle = NEUTRAL_PWM;
        newSteering = NEUTRAL_PWM;
        bUpdateFlags |= THROTTLE_FLAG | STEERING_FLAG;
        Serial.println("Stop and steer straight.");
        break;
    }
  }

  if(bUpdateFlags & THROTTLE_FLAG)
    {
      if(servoThrottle.readMicroseconds() != newThrottle)
      {
        servoThrottle.writeMicroseconds(newThrottle);
      }
    }
    
    if(bUpdateFlags & STEERING_FLAG)
    {
      if(servoSteering.readMicroseconds() != newSteering)
      {
        servoSteering.writeMicroseconds(newSteering);
      }
    }
    bUpdateFlags = 0;
}

void flash()
{
  output = !output;
  digitalWrite(LED_BUILTIN, output);
}