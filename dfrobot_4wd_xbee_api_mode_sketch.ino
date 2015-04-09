/*
 *
 * IMPORTANT: Select “Arduino Duemilanove w/ATMega328” as the hardware
 *
 * A program to remotely control a 4 wheel drive robot. 
 *
 * Requires:
 * - Sparkfun XBee shield (series 1)
 * - Sparkfun XBee USB explorer
 * - dfrobot 4WD platform
 * - dfrduino romeo all-in-one controller
 * - 3 x dfrobot adjustable infrared sensors
 *
 * Now supports autonomous mode via the front-end GUI application - but button-activated autonomous mode is broken for some reason.
 *
 * Analog values on A7 for buttons S2-S7:
 *  No key=391-393
 *  Key 2=312-313
 *  Key 3=322-325
 *  Key 4=331-332
 *  Key 5=352-353
 *  Key 6=384-386
 *  Key 7=386-388
 *
 */

#include <XBee.h>

// Commands I understand

#define CMD_MOTOR_STOP			0 
#define CMD_MOTOR_FORWARD		1 
#define CMD_MOTOR_BACKWARDS		2 
#define CMD_MOTOR_LEFT			3 
#define CMD_MOTOR_RIGHT			4 
// #define CMD_PANTILT_LEFT		5 
// #define CMD_PANTILT_RIGHT		6 
// #define CMD_PANTILT_UP		7 
// #define CMD_PANTILT_DOWN		8 
#define CMD_PING			9
#define CMD_AUTONOMOUS_MODE_ON          10
#define CMD_AUTONOMOUS_MODE_OFF         11

// Max is 255 - controls the frequency of the PWM signal, which controls the motor speed.
#define SPEED				200 // Default speed for autonomous mode


int AUTONOMOUS_MODE_TOGGLE_PIN = 7 ;  // Button #2 - bottom left - uses Analog pin 7. We're looking for a value of 150 to indicate it was pressed.
int LED_PIN = 13 ;                    // We use the LED rather cryptically to indicate problems

// Arduino PWM pins
int E1 = 5 ;    //M1 Speed Control
int E2 = 6 ;    //M2 Speed Control
int M1 = 4 ;    //M1 Direction Control
int M2 = 7 ;    //M2 Direction Control

// Infrared sensor pins
int INFRARED_1 = 9 ;
int INFRARED_2 = 10 ;
int INFRARED_3 = 11 ;

int isMoving = false ;
int autonomousMode = true ;
int previousS7ButtonState = 0 ;

XBee xbee = XBee() ;

void setup(void) { 
  pinMode(LED_PIN, OUTPUT) ;
//  pinMode(AUTONOMOUS_MODE_TOGGLE_PIN, INPUT) ;
  
  // Initialise the infrared sensors as inputs
  pinMode(INFRARED_1,INPUT) ;
  pinMode(INFRARED_2,INPUT) ;
  pinMode(INFRARED_3,INPUT) ;

  // Initialise the digital pins as outputs
  pinMode(E1, OUTPUT) ; 
  pinMode(E2, OUTPUT) ; 
  pinMode(M1, OUTPUT) ; 
  pinMode(M2, OUTPUT) ;
  
  // previousS7ButtonState = digitalRead(AUTONOMOUS_MODE_TOGGLE_PIN) ; // Get the initial state of button S7
  previousS7ButtonState = analogRead(AUTONOMOUS_MODE_TOGGLE_PIN) == 150 ; // Get the initial state of button S7
  
  // flashLed(LED_PIN, 4, 250);
  
 xbee.begin(9600) ; 
 stop() ; // Make sure it is in a known state to start off .
 
//  Serial.begin(9600) ; // Comment me out when using the XBee
//  Serial.println("Begins") ;
} 

//void loop(void) {
//    advance(255, 255);
//}

void loop(void) {

//  int s7ButtonState2 = analogRead(AUTONOMOUS_MODE_TOGGLE_PIN) == 150 ; // Get the current state of button S7
//  Serial.print("Button state: ") ;
//  Serial.println(s7ButtonState2) ;
//  delay(250) ;
//  return ;
  
   // Read command from remote XBee
  XBeeResponse response = XBeeResponse();
  Rx16Response rx16 = Rx16Response(); // Response data will be stuffed in here
  xbee.readPacket(); // Read some data

  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16); // Put the response data in rx16
      int command = rx16.getData(0);
      int data = rx16.getData(1) ; 
      switch(command) {
      case CMD_MOTOR_STOP:
        stop();
        break ;
      case CMD_MOTOR_FORWARD:
        if(data > 0)
          advance(data, data);
        else
          stop() ;
        break ;
      case CMD_MOTOR_BACKWARDS:
        if(data > 0)
          back_off(data, data);
        else
          stop() ;
        break ;
      case CMD_MOTOR_LEFT:
        if(data > 0)
          turn_right(data, data); // I wired the motors wrong - left is right and right is left.
        else
          stop() ;
        break ;
      case CMD_MOTOR_RIGHT:
        if(data > 0)
          turn_left(data, data);
        else
          stop() ;
        break ;
      case (CMD_AUTONOMOUS_MODE_ON):
        autonomousMode = true ;
        break ;
      case (CMD_AUTONOMOUS_MODE_OFF):
        autonomousMode = false ;
        stop() ;
        break ;
      
      }
     //  flashLed(LED_PIN, 4, 100);
    } else {
      // flashLed(LED_PIN, 8, 100);
    }
  } else if(xbee.getResponse().isError()) {
    // flashLed(LED_PIN, 1, 500);
  } else {
//    flashLed(LED_PIN, 2, 100);
//    delay(500) ;
  }
  
   // We use the DFRobot Romeo's S7 button - whose state is obtained via digital pin 3) to toggle autonomous mode. But beware that as the robot doesn't (yet) send state information
   // back to the PC running the controller application, the app will be out of sync. All this means is you can't use the app to enable autonomous mode and the S7 button to disable it.
  // int s7ButtonState = digitalRead(AUTONOMOUS_MODE_TOGGLE_PIN) ;
  int s7ButtonState = analogRead(AUTONOMOUS_MODE_TOGGLE_PIN) == 150 ; // Get the current state of button S7
  if(s7ButtonState != previousS7ButtonState) {
    delay(50) ; // Debounce delay
    if(s7ButtonState == (analogRead(AUTONOMOUS_MODE_TOGGLE_PIN) == 150)) {// Debounce check
      autonomousMode != autonomousMode ;
      previousS7ButtonState = s7ButtonState ;
    }
  }

  /*
   * A visual cue that we are in autonomous mode
   */
  if(autonomousMode)
    digitalWrite(LED_PIN, HIGH) ;
  else
    digitalWrite(LED_PIN, LOW) ;
    
  /*
   * In a nutshell, the autonomous mode logic is as follows:
   * - Check if we're blocked by an obstacle (ie read the sensors)
   * - If so, rotate a bit and stop
   * - If not, then if we're not moving then go forward
   */
  if(autonomousMode) {

    // Read sensors - LOW means obstacle detectted, HIGH means all clear
    int willCollide = false ;
    if(digitalRead(INFRARED_1) == LOW ||  digitalRead(INFRARED_2) == LOW || digitalRead(INFRARED_3) == LOW)
      willCollide = true ;
    
    if(willCollide) {
      turn_left(SPEED, SPEED) ;
      delay(1000) ;
      stop() ; // we could go on indefinitely trying to fight our way out of a hole here, but instead we stop and wait for the next iteration of loop() so that any pending wireless commands can be consumed.
    } else if(! isMoving) {
      advance(SPEED, SPEED) ; // Not moving, lets go!
    }
  }
}

void stop(void) {
          if(isMoving == 0)
            return ;
          digitalWrite(E1,LOW);   
          digitalWrite(E2,LOW);
          isMoving = 0 ;
}

void advance(char a,char b) {
          analogWrite (E1,a);      //PWM Speed Control
          digitalWrite(M1,HIGH);    
          analogWrite (E2,b);    
          digitalWrite(M2,HIGH);
          // Serial.println("Advanced") ;
          isMoving = 1 ;
}

void back_off (char a,char b) {
          analogWrite (E1,a);
          digitalWrite(M1,LOW);   
          analogWrite (E2,b);    
          digitalWrite(M2,LOW);
          // Serial.println("Backed off") ;
          isMoving = 1 ;
}

void turn_left(char a,char b) {
          analogWrite (E1,a);
          digitalWrite(M1,LOW);    
          analogWrite (E2,b);    
          digitalWrite(M2,HIGH);
//          Serial.println("Turned left") ;
          isMoving = 1 ;
}

void turn_right(char a,char b) {
          analogWrite (E1,a);
          digitalWrite(M1,HIGH);    
          analogWrite (E2,b);    
          digitalWrite(M2,LOW);
//          Serial.println("Turned right") ;
          isMoving = 1 ;
}

void flashLed(int pin, int times, int wait) {
    
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }
}

