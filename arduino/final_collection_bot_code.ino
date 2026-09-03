/*
 * ARA Main Controller v7.1
 */

#include <Servo.h>
#include <NewPing.h>

//servo pins
const int LEFT_DRIVE_PIN = 11;
const int RIGHT_DRIVE_PIN = 12;
const int CLAW_SERVO_PIN = 3;

//sensor pins
const int SONAR_TRIG_PIN = 8;
const int SONAR_ECHO_PIN = 9;
const int LEFT_IR_PIN = A0;
const int RIGHT_IR_PIN = A1;

//objects
Servo servoLeft;
Servo servoRight;
Servo clawServo;
NewPing sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN, 40); //40cm max distance it will read to make it get faster response times

//states of being for Sparkbot
enum State { HALTED, SEEKING, RETURNING_HOME, DROPPING };
State currentState;

//movement servo values
const int LEFT_DRIVE_FORWARD_US = 1600;
const int LEFT_DRIVE_STOP_US = 1500;
const int LEFT_DRIVE_REVERSE_US = 1400;
const int RIGHT_DRIVE_FORWARD_US = 1400;
const int RIGHT_DRIVE_STOP_US = 1500;
const int RIGHT_DRIVE_REVERSE_US = 1600;

//claw servo values
const int CLAW_OPEN_ANGLE = 90;
const int CLAW_CLOSED_ANGLE = 0;

//sensor values
const int OBSTACLE_DISTANCE_CM = 12; //12cm stopping distance
//lower value = more sensitive to white
const int LEFT_LINE_THRESHOLD = 980; 
const int RIGHT_LINE_THRESHOLD = 980; 

//deploy state delay times
const int DEPLOY_REV_MS = 2000; //back up for 2 seconds
const int TURN_90_MS = 500;
const int AVOID_REV_MS = 500; //how long to back up when avoiding

void setup() {
  Serial.begin(9600);

  //attach servos
  servoLeft.attach(LEFT_DRIVE_PIN);
  servoRight.attach(RIGHT_DRIVE_PIN);
  clawServo.attach(CLAW_SERVO_PIN);

  goStop();
  currentState = HALTED; //wait for pi's command
  
  Serial.println("READY");
}

void loop() {
  //sensor reflexes (this runs first before anything
  int distance = sonar.ping_cm();
  if (distance == 0) distance = 255; // 0 means out of range so set to far away

  //values for when seeing white lines with different sensors
  int leftIR = analogRead(LEFT_IR_PIN);
  int rightIR = analogRead(RIGHT_IR_PIN);
  bool leftSeesWhite = (leftIR < LEFT_LINE_THRESHOLD);
  bool rightSeesWhite = (rightIR < RIGHT_LINE_THRESHOLD);

  //exceptions for the sensors
  // If we are in HALTED or DROPPING sensors don't stop us
  if (currentState == HALTED || currentState == DROPPING) {
    runStateLogic(); //run the normal logic and skip all sensors
    return; //exit the loop early
  }

  //avoiding lines
  //runs in SEEKING, but is ignored in RETURNING_HOME
  if (currentState != RETURNING_HOME) {
    if (leftSeesWhite && rightSeesWhite) {
      //when both see white back up and turn 180
      Serial.println("REFLEX: Both lines! Backing up 180.");
      goReverse();
      delay(AVOID_REV_MS);
      goRight(); //spin 180
      delay(TURN_90_MS * 2);
      goStop();
      return; //reflex done restart loop
    } else if (leftSeesWhite) {
      //when left sees white back up and turn right
      Serial.println("REFLEX: Left line! Turning Right.");
      goReverse();
      delay(AVOID_REV_MS);
      goRight(); //turn 90
      delay(TURN_90_MS);
      goStop();
      return; //reflex done restart loop
    } else if (rightSeesWhite) {
      //when right sees white back up and turn left
      Serial.println("REFLEX: Right line! Turning Left.");
      goReverse();
      delay(AVOID_REV_MS);
      goLeft(); //turn 90
      delay(TURN_90_MS);
      goStop();
      return; //reflex done restart loop
    }
  }

  //avoiding walls
  //this runs in SEEKING and RETURNING_HOME
  //add 'distance > 0' to ignore bad 0cm readings
  if (distance < OBSTACLE_DISTANCE_CM && distance > 0) {
    Serial.println("REFLEX: Wall! Backing up 90.");
    goReverse();
    delay(AVOID_REV_MS);
    goRight(); //turn 90
    delay(TURN_90_MS);
    goStop();
    return; //reflex done restart loop
  }

  //main state logic
  //if no reflexes were triggered run the normal state logic
  runStateLogic();
}

//switch statement function
void runStateLogic() {
  switch (currentState) {
    case HALTED:
      //wait for 'I' (Initiate) command from the pi.
      if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'I') {
          Serial.println("State: DEPLOY_FLAG (Simplified)");
          
          //deploy sequence
          //open claw drop flag
          clawServo.write(CLAW_OPEN_ANGLE);
          delay(1000); //wait for claw to open

          //back straight up
          Serial.println("Backing up...");
          goReverse();
          delay(DEPLOY_REV_MS); // 2 seconds
          
          //spin around (180 degrees)
          Serial.println("Spinning around...");
          goRight(); // Spin right
          delay(TURN_90_MS * 2); // 1 second
          
          //stop and enter main loop
          goStop();
          Serial.println("SEEK"); //tell pi to start hunting
          currentState = SEEKING;
          
          Serial.println("Deploy complete. Entering main loop.");
        }
      }
      break;

    case SEEKING:
      //listen for commands
      handlePiMovement();
      
      //apply continuous pressure to claw servo opened state
      clawServo.write(CLAW_OPEN_ANGLE);
      
      break;

    case RETURNING_HOME:
      //listen for commands
      handlePiMovement();

      //apply continuous pressure to claw servo closed state
      clawServo.write(CLAW_CLOSED_ANGLE);
      break;

    case DROPPING:
      //transition states
      //handlePiMovement() sets state back to SEEKING
      break;
  }
}

//listen to pi for movement commands
void handlePiMovement() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    //handle grab
    if (command == 'G' && currentState == SEEKING) {
      currentState = RETURNING_HOME;
      goStop();
      //main loop will apply continuous pressure
      Serial.println("HOME"); //tell pi to look for yellow

    //handle drop
    } else if (command == 'D' && currentState == RETURNING_HOME) {
      currentState = DROPPING;
      goStop();
      clawServo.write(CLAW_OPEN_ANGLE); //drop object
      delay(1000); //wait for claw to open

      //mission loop
      //back up
      goReverse();
      delay(1000); //back up for 1 second
      //turn 180
      goRight();
      delay(TURN_90_MS * 2); //1 second turn
      //drive away
      goForward();
      delay(3000); //drive away for 3 seconds
      //stop
      goStop();
      
      //clear the buffer and tell the pi to seek
      clearSerialBuffer();
      Serial.println("SEEK");
      currentState = SEEKING;

    //basic homing/seeking commands
    } else if (command == 'F') {
      goForward();
    } else if (command == 'L') {
      goLeft();
    } else if (command == 'R') {
      goRight();
    } else if (command == 'S') {
      goStop();
    }
  }
}


//avoids bad commands from serial clutter
void clearSerialBuffer() {
  Serial.println("Clearing serial buffer...");
  while (Serial.available() > 0) {
    Serial.read(); //read and discard
  }
}


//movement functions
void goStop() {
  servoLeft.writeMicroseconds(LEFT_DRIVE_STOP_US);
  servoRight.writeMicroseconds(RIGHT_DRIVE_STOP_US);
  Serial.println("Stopping");
}
void goForward() {
  servoLeft.writeMicroseconds(LEFT_DRIVE_FORWARD_US);
  servoRight.writeMicroseconds(RIGHT_DRIVE_FORWARD_US);
  Serial.println("Going Forward");
}
void goReverse() {
  servoLeft.writeMicroseconds(LEFT_DRIVE_REVERSE_US);
  servoRight.writeMicroseconds(RIGHT_DRIVE_REVERSE_US);
  Serial.println("Going Reverse");
}
void goLeft() { //pivot left
  servoLeft.writeMicroseconds(LEFT_DRIVE_STOP_US);
  servoRight.writeMicroseconds(RIGHT_DRIVE_FORWARD_US);
  Serial.println("Turning Left");
}
void goRight() { //pivot right
  servoLeft.writeMicroseconds(LEFT_DRIVE_FORWARD_US);
  servoRight.writeMicroseconds(RIGHT_DRIVE_STOP_US);
  Serial.println("Turning Right");
}
