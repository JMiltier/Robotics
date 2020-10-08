#include <sparki.h> // include the sparki library
#include "pitches.h" // include a list of pitches

// Set up some global variables with default values to be replaced during operation
const int threshold = 700; // IR reading threshold to detect whether there's a black line under the sensor
int lineLeft = 0;   // measure the left IR sensor
int lineRight = 0;  // measure the right IR sensor
int lineCenter = 0; // measure the center IR sensor
int hasObject = 0;
int cm = 0;
String state = "undefined";

// notes in the melody:
int melody[] = { 
  NOTE_E7, NOTE_E7, 0, NOTE_E7, 
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = { 
  12, 12, 12, 12, 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12
};

void RGBcolor(int code){
    switch(code){
      case NOTE_E7: sparki.RGB(RGB_ORANGE); break;
      case NOTE_G7: sparki.RGB(RGB_GREEN); break;
      case NOTE_G6: sparki.RGB(RGB_INDIGO); break;
      case NOTE_C7: sparki.RGB(RGB_VIOLET); break;
      case 0: sparki.RGB(RGB_OFF); break;
      default: break;
    }
}

//victory song at end (Mario), !cannot add more due to memory!
void sing() {
  // play each note in the arrays
  for (int thisNote = 0; thisNote < 13; thisNote++) {
 
    // calculate the note duration as 1 second divided by note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];
    sparki.beep(melody[thisNote],noteDuration);
    RGBcolor(melody[thisNote]);
 
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    delay(noteDuration*0.8);       
    sparki.RGB(RGB_OFF);
    delay(noteDuration*0.8);
    // stop the tone playing:
    sparki.noBeep();
  }
}

// reads initialization of sensors
void readSensors() {
  lineLeft = sparki.lineLeft(); // Replace with code to read the left IR sensor
  lineRight = sparki.lineRight(); // Replace with code to read the right IR sensor
  lineCenter = sparki.lineCenter(); // Replace with code to read the center IR sensor
  cm = sparki.ping(); // Replace with code to read the distance sensor
}

// code setup, to run once:
void setup() {
  sparki.RGB(10,10,10); // Turn on the White LED (For "boot")
  sparki.beep(220, 300);
  delay(300);
  sparki.beep(880, 300);
  sparki.servo(SERVO_CENTER); //Center the ultrasonic sensor
  sparki.gripperOpen(); // Open the gripper
  delay(1000); // Give the motor time to open the gripper (1 second, there should only be .1 seconds remaining from last use)
  sparki.gripperStop(); // 5 seconds should be long enough
  hasObject = 0;
}

//show current sensors status and state of sparki (search, etc.)
void displaySensorsAndState(){
  sparki.clearLCD(); // wipe the screen
 
  sparki.print("LL:"); // show left line sensor on screen
  sparki.print(lineLeft);
 
  sparki.print(" LC:"); // show center line sensor on screen
  sparki.print(lineCenter);
 
  sparki.print(" LR:"); // show right line sensor on screen
  sparki.println(lineRight);
 
  sparki.print("Ping: "); // ultrasonic ranger on screen
  sparki.print(cm);
  sparki.println(" cm");

  sparki.print("Object State: "); //check state of hasObject
  sparki.println(hasObject);
  sparki.println();

  sparki.println(String("state = ") + state); //shows the state of current action
 
  sparki.updateLCD(); // display all of the information written to the screen
}

//prevent Sparki from falling off the table (may be issue with detecting the track vs. table edge)
//!!!CURRENTLY NOT IN USE!!!
void edgeDetection(){
  int edgeLeft   = sparki.edgeLeft();   // measure the left edge IR sensor
  int edgeRight  = sparki.edgeRight();  // measure the right edge IR sensor
 
  int threshold = 700; // if below this value, no surface underneath
 
  if (edgeLeft < threshold) // if no surface underneath left sensor
  {
    sparki.moveBackward(5);
    sparki.moveLeft(45); // turn right
  }
 
  if (edgeRight < threshold) // if no surface underneath right sensor
  {
    sparki.moveBackward(5);
    sparki.moveLeft(45); // turn left
  }
 
  sparki.moveForward(); // move forward
  delay(100); // wait 0.1 seconds
}

//Actively searches for object (NOTICE: singular object)
void objectSearch() {
  if(cm != -1) // make sure its not too close or too far, then rotate left
    { 
      sparki.moveLeft();
      state = "Searching for     Object";
        if(cm <= 30 && cm >4) // if the object is nearby: 30 cm or less
        {
            sparki.RGB(RGB_RED); // turn the light red
            state = "Approaching     Object";
            sparki.moveForward();
        }
        if(cm <= 4 && cm >= 1)
        {
              sparki.RGB(RGB_ORANGE); // turn the light orange
              delay(100);
              sparki.moveStop();
              state = "Gripping Object";
              displaySensorsAndState();
              sparki.gripperClose();
              for (int i = 0; i < 8; i++){ 
                sparki.beep(300, 300); 
                delay(50); 
                sparki.beep(80, 300);
                delay(50);
                sparki.noBeep();
                delay(100);
              }
              sparki.gripperStop();
              hasObject = 1;
              state = "180 degree     turn";
              displaySensorsAndState();
              sparki.moveRight(180);
         }
    }
}

void searchForLine(){
  state = "Searching for     Line";
  //if line is approached head on, rotate at 45 degrees
  if ((abs(lineLeft - lineRight) < 200) && (lineCenter < threshold)){
      sparki.moveRight(35);
      delay(1000);
  //before threshold is found, sparki will move forward
  }else if(lineCenter > threshold) sparki.moveForward();  
  //once sparki finds the line based on the center being less than the threshold
  //movement will stop and beep to signify line is found
  else{
    sparki.moveStop();
    sparki.beep(110, 500);  
    hasObject = 2;
  }
}

//follows the line to "START" line
void lineFollow(){  
    readSensors();
    state = "Following Line";
    displaySensorsAndState();
    
    if ( lineLeft < threshold || ((lineRight-lineLeft) > 100)) // if line is below left line sensor
    {  
      //these functions followed the line much better than the moveLeft function
      sparki.motorRotate(MOTOR_LEFT, DIR_CCW, 0);
      sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 100);
      //sparki.moveLeft(); // turn left
    }
   
    if ( lineRight < threshold || ((lineLeft-lineRight) > 100)) // if line is below right line sensor
    {  
      //these functions followed the line much better than the moveRight function
      sparki.motorRotate(MOTOR_LEFT, DIR_CCW, 100);
      sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 0);
      //sparki.moveRight(); // turn right
    }
   
    // if the center line sensor is the only one reading a line
    if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
    {
      sparki.moveForward(); // move forward
    }

    if ( (lineLeft < threshold) && (lineRight < threshold) && (lineCenter < threshold))
    {
      sparki.beep(250,300);delay(300);sparki.beep(300,300);
      sparki.moveStop();
      state = "Releasing Object";
      hasObject = 3;
      displaySensorsAndState();
      sparki.gripperOpen();
      delay(1500);
      sparki.moveForward();
      sparki.gripperStop();
      delay(1000); //wait 1 seconds to complete operations
      sparki.moveBackward();
      delay(1000);
      
  }
}

//finishing moves
void finished(){
  //tells the user that the work is done:
  readSensors();
  state = "finished";
  sparki.motorRotate(MOTOR_LEFT, DIR_CCW, 100);
  sparki.motorRotate(MOTOR_RIGHT, DIR_CCW, 100);
  delay(250);
  sparki.motorRotate(MOTOR_LEFT, DIR_CW, 100);
  sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 100);
  delay(500);
  sparki.motorRotate(MOTOR_LEFT, DIR_CCW, 100);
  sparki.motorRotate(MOTOR_RIGHT, DIR_CCW, 100);
  delay(500);
  sparki.motorRotate(MOTOR_LEFT, DIR_CW, 100);
  sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 100);
  delay(250);
  sparki.moveStop();
  sing();
  displaySensorsAndState();
  sparki.RGB(RGB_OFF);
  hasObject = 4;
}

void loop() {
  // main code, to run repeatedly:
  sparki.RGB(0,30,0);

  if (hasObject == 0) objectSearch();
  //edgeDetection();
  else if (hasObject == 1) searchForLine();
  else if (hasObject == 2) lineFollow();
  else if (hasObject == 3) finished();
  else{ sparki.moveStop(); NULL;}
  
  delay(100); // wait 0.1 seconds (100 milliseconds)
  readSensors();// Read sensors once per loop() call

  displaySensorsAndState();
  delay(100); // Only run controller at 10Hz
}
