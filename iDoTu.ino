//Sparkfun tilt sensor simple sketch code with debouncing
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define DEBUG

#define TIMER 800; // 20 mins * 60 / 1.5

//#define TIMER 10;

typedef enum {
  STATE_TIMER = 0,  // counting mins left
  STATE_BLINKLED = 1, // blink LED
  STATE_RESETTIMER = 2, // reset timer when drank water
} state_t;

state_t state = STATE_TIMER;

// these constants won't change:
const int tiltSensor = 0; // tilt sensor is connected to pin 0
const int ledPin = 1;      // led connected to pin 1

// these variables will change:
int sensorReading;      // variable to store the value read from the sensor pin
int ledState = LOW;       // variable used to store the last LED status, to toggle the light
int tiltState;            // the current reading from the input pin
int lastTiltState = HIGH; // the previous reading from the input pin

boolean drankWater = false;
long timeLeft = TIMER; 

long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
 wdt_disable(); // disable watchdog to prevent reset forever
 power_adc_disable(); // disable adc
 //power_spi_disable(); // disable spi
 
 Serial.begin(9600);
 Serial.println("Starting...");
 Serial.println(state);
 pinMode(ledPin, OUTPUT); // declare the ledPin as as OUTPUT
 pinMode(tiltSensor, INPUT); // declare the tiltSensor as as INPUT
}

void loop() {
  switch(state) {
    case STATE_TIMER:
      Serial.println(state);
      checkTilt();
      sleepNow();
      if(drankWater) {
        state = STATE_RESETTIMER;
        Serial.println(state);
      }
      else if (timeLeft == 0) {
        state = STATE_BLINKLED;
        Serial.println(state);
      }
      else {
        state = STATE_TIMER;
        Serial.println(state);
      }
      break;
    case STATE_BLINKLED:
      wdt_disable();  // disable watchdog
      checkTilt();
      if(drankWater) {
        state = STATE_RESETTIMER;
        Serial.println(state);
      }
      else {
        state = STATE_BLINKLED;
        Serial.println(state);
        blinkLED();
      }
      break;
    case STATE_RESETTIMER:
      resetTimer();
      state = STATE_TIMER;
      Serial.println(state);
      break;
    default:
      break;
  }
}

void blinkLED() {
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(100);
}

void resetTimer() {
  timeLeft = TIMER;
}

void countTimeLeft() {
  timeLeft--;
  Serial.println(timeLeft);
}

void checkTilt() {
  // read the sensor and store it in the variable sensorReading:
  sensorReading = digitalRead(tiltSensor);    
  
  // check to see if the sensor was tilted
  // (i.e. the input went from HIGH to LOW), and you've waited 
  // long enough since the last change to ignore any noise:  

  // If the tilt changed, due to noise or tilting: 
  if (sensorReading != lastTiltState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
   if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    
    // if the button state has changed:
    if (sensorReading != tiltState){
      tiltState = sensorReading;
      
    // if the sensor reading is low
      if (sensorReading == LOW) {
        // make the status of the ledPin to go on:
        drankWater = true;
      }
      // otherwise if it is high
      else if (sensorReading == HIGH){
        // make the status of the ledPin to stay off:
        drankWater = false;
      }
    }
   }
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastTiltState = sensorReading; 
  // delay to avoid overloading the serial port buffer:
  delay(100);
}

// watchdog interrupt
ISR (WDT_vect) {
  countTimeLeft();
}

//void sleepNow() {
//    /* Clear the reset flag. */
//  MCUSR &= ~(1<<WDRF);
//  
//  /* In order to change WDE or the prescaler, we need to
//   * set WDCE (This will allow updates for 4 clock cycles).
//   */
//  WDTCSR |= (1<<WDCE) | (1<<WDE);
//
//  /* set new watchdog timeout prescaler value */
//  WDTCSR = 1<<WDP0 | 1<<WDP2; /* 0.5 seconds */
//  
//  /* Enable the WD interrupt (note no reset). */
//  WDTCSR |= _BV(WDIE);
//
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  sleep_enable();
//
//  Serial.println("Sleep...");
//  sleep_mode();
//
//  Serial.println("Disable sleep mode");
//  sleep_disable();
//}


void sleepNow() {
  setup_watchdog(5); // setup watchgod to go off after 8 seconds
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode(); // go to sleep mode
}

// Watchdog timeout values
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms, 6=1sec, 7=2sec, 8=4sec, 9=8sec
// Used from http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int ii) {
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  
  MCUSR &= ~(1<<WDRF);
  // Start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // Set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}
