#include <Arduino.h>
#include <Servo.h>
#include <MsTimer2.h>

// Assign your channel in pins
#define THROTTLE_IN_ESTOP_PIN 3
#define STEERING_IN_PIN 4

// Assign your channel out pins
#define THROTTLE_OUT_PIN 5
#define STEERING_OUT_PIN 6

#define NEUTRAL_PWM 1500
#define FULL_FORWARD_PWM 2000
#define FULL_REVERSE_PWM 1000
#define FULL_RIGHT_PWM 2000
#define FULL_LEFT_PWM 1100

// Servo objects 
static Servo servoThrottle;
static Servo servoSteering;

// channels have new signals flags
#define THROTTLE_ESTOP_FLAG 1
#define STEERING_FLAG 2

// These variables are for manual mode
// holds the update flags defined above
static volatile uint8_t bUpdateFlagsShared = 0;
// shared variables are updated by the ISR and read by loop.
static volatile uint16_t unThrottleInShared = 0;
static volatile uint16_t unSteeringInShared = 0;
// To record the rising edge of a pulse in the calcInput functions
static uint32_t ulThrottleStart = 0;
static uint32_t ulSteeringStart = 0;

// blink
static boolean output = HIGH;
static boolean autoMode = true;

void flash();
uint16_t getSteeringByPercent(int8_t percent);
uint16_t getThrottleByPercent(int8_t percent);
void switchToManualMode(boolean& autoMode);
void switchToAutoMode(boolean& autoMode);
void calcThrottle();
void calcSteering();

void setup()
{
  Serial.begin(115200);

  // input pins
  pinMode(THROTTLE_IN_ESTOP_PIN, INPUT);
  pinMode(STEERING_IN_PIN, INPUT);
  
  // attach servo objects  
  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);

  if (autoMode) 
    switchToAutoMode(autoMode);
  else
    switchToManualMode(autoMode);

  // Interrupt for throttle pin is always present 
  // In Auto Mode it serves as the safety cutoff switch
  attachInterrupt(THROTTLE_IN_ESTOP_PIN, calcThrottle,CHANGE); 
  
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  MsTimer2::set(500, flash);
  MsTimer2::start();
}

void loop()
{
  // these percents are for auto mode
  static int8_t throttlePercent = 0; // -100 to 100, < 0 is backwards
  static int8_t steeringPercent = 0; // -100 to 100, < 0 is left
  static boolean movementEnabled = false;

  uint8_t bUpdateFlags = 0;
  uint16_t newThrottle = 0;
  uint16_t newSteering = 0;
  int incomingByte = 0; // for incoming serial data

  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    //Serial.println(incomingByte, DEC);

    if (autoMode) 
    {
      switch (incomingByte) 
      {
        case 'r':
          steeringPercent = min(100, steeringPercent + 5);
          newSteering = getSteeringByPercent(steeringPercent);
          bUpdateFlags |= STEERING_FLAG;
          Serial.printf("Turning right by 5%% more. [%d%%] PWM = %d\n", steeringPercent, newSteering);
          break;
        case 'l':
          steeringPercent = max(-100, steeringPercent - 5);
          newSteering = getSteeringByPercent(steeringPercent);
          bUpdateFlags |= STEERING_FLAG;
          Serial.printf("Turning left by 5%% more. [%d%%] PWM = %d\n", steeringPercent, newSteering);
          break;
        case 'f':
          throttlePercent = min(100, throttlePercent + 5);
          newThrottle = getThrottleByPercent(throttlePercent);
          bUpdateFlags |= THROTTLE_ESTOP_FLAG;
          Serial.printf("Increase throttle 5%%. [%d%%] PWM = %d\n", throttlePercent, newThrottle);
          break;
        case 'b':
          throttlePercent = max(-100, throttlePercent - 5);
          newThrottle = getThrottleByPercent(throttlePercent);
          bUpdateFlags |= THROTTLE_ESTOP_FLAG;
          Serial.printf("Decrease throttle 5%%. [%d%%] PWM = %d\n", throttlePercent, newThrottle);
          break;
        case 's':
          steeringPercent = 0;
          newSteering = NEUTRAL_PWM;
          bUpdateFlags |= STEERING_FLAG;
          Serial.println("Neutral steering.");
          break;
        case 'n':
          throttlePercent = 0;
          newThrottle = getThrottleByPercent(throttlePercent);
          bUpdateFlags |= THROTTLE_ESTOP_FLAG;
          Serial.printf("Neutral throttle. [0%%] PWM = %d\n", newThrottle);
          break;
        case 'm':
          Serial.println("Switching to Manual Mode.");
          throttlePercent = 0;
          steeringPercent = 0;
          switchToManualMode(autoMode);
          break;
      }
    } 
    else 
    {
      if (incomingByte == 'a') 
      {
        Serial.println("Switching to Auto Mode.");
        switchToAutoMode(autoMode);
      }
    }
  }

  // If manual mode, update controls from remote inputs
  if(not autoMode)
  {
    // check if any channels have a new signal
    if(bUpdateFlagsShared)
    {
      noInterrupts(); // turn interrupts off 
      bUpdateFlags = bUpdateFlagsShared;
      
      if(bUpdateFlags & THROTTLE_ESTOP_FLAG)
      {
        newThrottle = unThrottleInShared;
      }
      
      if(bUpdateFlags & STEERING_FLAG)
      {
        newSteering = unSteeringInShared;
      }
      
      bUpdateFlagsShared = 0;
      interrupts(); 
    }
  }
  else // autoMode 
  {
    // update the estop safety switch, movementEnabled, from remote throttle which must be engaged
    // for movement to happen
    if(bUpdateFlagsShared)
    {
      noInterrupts(); // turn interrupts off 
      
      if(bUpdateFlagsShared & THROTTLE_ESTOP_FLAG)
        movementEnabled = unThrottleInShared > 1700;
     
      bUpdateFlagsShared = 0;
      interrupts(); 
    }
  }

  // Only update controls if movement is enabled or manual mode is active.
  if(movementEnabled || not autoMode)
  {
    if(bUpdateFlags & THROTTLE_ESTOP_FLAG)
      {
        //Serial.printf("newThrottle = %d\n", newThrottle);
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
  }
  else // not movementEnabled && autoMode
  {
    // safety switch released and all movement is disabled
    servoThrottle.writeMicroseconds(NEUTRAL_PWM);
  }
}

// simple interrupt service routine
void calcThrottle()
{
  uint16_t tempThrottle;
  // if the pin is high, its a rising edge
  if(digitalRead(THROTTLE_IN_ESTOP_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    tempThrottle = (uint16_t)(micros() - ulThrottleStart);
    if (tempThrottle >= 1000 && tempThrottle <= 2000)
    {
      unThrottleInShared = tempThrottle;
      bUpdateFlagsShared |= THROTTLE_ESTOP_FLAG;
    }
  }
}
void calcSteering()
{
  uint16_t tempSteering;
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  { 
    ulSteeringStart = micros();
  }
  else
  {
    tempSteering = (uint16_t)(micros() - ulSteeringStart);
    if (tempSteering >= 1000 && tempSteering <= 2000)
    {
      unSteeringInShared = tempSteering;
      bUpdateFlagsShared |= STEERING_FLAG;
    }
  }
}

void flash()
{
  output = !output;
  digitalWrite(LED_BUILTIN, output);
}

uint16_t getSteeringByPercent(int8_t percent)
{
  return (percent * (int) (FULL_RIGHT_PWM - FULL_LEFT_PWM) / 2) / 100 + NEUTRAL_PWM;
}

uint16_t getThrottleByPercent(int8_t percent)
{
  return (percent * (int) (FULL_FORWARD_PWM - FULL_REVERSE_PWM) / 2) / 100 + NEUTRAL_PWM;
}

void switchToAutoMode(boolean& autoMode)
{
  // uninstall steering interrupt
  detachInterrupt(STEERING_IN_PIN);

  // reset manual mode variables
  bUpdateFlagsShared = 0;
  unThrottleInShared = 0;
  unSteeringInShared = 0;
  ulThrottleStart = 0;
  ulSteeringStart = 0;
  
  autoMode = true;

  // Set steering and throttle to neutral
  servoSteering.writeMicroseconds(NEUTRAL_PWM);
  servoThrottle.writeMicroseconds(NEUTRAL_PWM);
}

void switchToManualMode(boolean& autoMode)
{
  // Set steering and throttle to neutral
  servoSteering.writeMicroseconds(NEUTRAL_PWM);
  servoThrottle.writeMicroseconds(NEUTRAL_PWM);

  autoMode = false;

  // attach the steering interrupt
  attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE); 
}
