#include <Arduino.h>
#include <Pixy2.h>
#include <SPI.h>
#include <Servo.h>

// Servo connection

// Constants
int servoLimit[] = {151, 31};
int servoP1Pin = 9;
int servoP2Pin = 10;
int servoP3Pin = 6;
float centerLength = 0.075;
float armLength = 0.03;
double plateDiameter = 0.35;
// Set to the edge of the plattform
long xAxis[2] = {63, 263}; // Servo P1 and P2 should point to the top of the plattform
long yAxis[2] = {6, 206};
double angelOfMotionLimit[] = { -10, 10};
int cameraFPS = 20;
float xSpeedArray[3];
float ySpeedArray[3];
float lastPosX;
float lastPosY;
long timer1;
long timeout = 1000/cameraFPS;
long K1 = 0.82*50; // 50 
long K2 = 0.69*20;


float yBallSetpoint = 0; // set to a value between 0 and 255
float yBallPosition;
float rollAngle;
//double rollOut;

float xBallSetpoint = 0; // set to a value between 0 and 255
float xBallPosition;
//double pitchOut;
float pitchAngle;

// This is the main Pixy object
Pixy2 pixy;

// This is the servo object
Servo P1Servo;
Servo P2Servo;
Servo P3Servo;



long getPosition(uint16_t blocks);

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init(); // Initilize the pixy object
  // Ready the servos
  P1Servo.attach(servoP1Pin);
  delay(1000);
  P2Servo.attach(servoP2Pin);
  delay(1000);
  P3Servo.attach(servoP3Pin);
  delay(1000);
}



void loop() {


  static int i = 0;
  // Servo test comment the underlying code after servo test
   //setServoPosition(float pitchAngle, float rollAngle, float offset) 
  //setServoPosition(0, 0, 0);
  //delay(2000);
  
  uint16_t blocks;
  // grab blocks!
  blocks = pixy.ccc.getBlocks();


  // Find the position of the largest object in the specified range
  if (blocks && isTimerExpired(timer1)) {
    //Serial.println(millis());
    // Start timer for the next execution
    timer1 = startTimer(timeout);
    //Serial.println("At least one block was found");
    long xPos;
    long yPos;
    float xSpeed;
    float ySpeed;

    // Get the position of the largest object
    getPosition(blocks, &xPos, &yPos);
    i++;

    // Scale the position from pixels to meter
    scalePosition(&xPos, &yPos, &xBallPosition, &yBallPosition);
    // Calculate the speed
    xSpeed = (xBallPosition - lastPosX)*1000.0/timeout;
    //Serial.println(xSpeed);    
    ySpeed = (yBallPosition - lastPosY)*1000.0/timeout;
    
    /*Serial.print("Ball x pos: ");
    Serial.println(xBallPosition);
    
    Serial.print("Ball y pos: ");
    Serial.println(yBallPosition);*/
    
    // Apply the filter for speed and calculate the filtered speed
    ySpeedArray[2] = ySpeedArray[1];
    ySpeedArray[1] = ySpeedArray[0];
    ySpeedArray[0] = ySpeed;

    ySpeed = 1/3.0*ySpeedArray[2] + 1/3.0*ySpeedArray[1] + 1/3.0*ySpeedArray[0];

    xSpeedArray[2] = xSpeedArray[1];
    xSpeedArray[1] = xSpeedArray[0];
    xSpeedArray[0] = xSpeed;
    xSpeed = 1/3.0*xSpeedArray[2] + 1/3.0*xSpeedArray[1] + 1/3.0*xSpeedArray[0];


    // Set the setpoint for the ball, creates a circle
    yBallSetpoint = 2*sin(millis()/500);
    xBallSetpoint = 2*cos(millis()/500);

    pitchAngle = -K2*ySpeed - K1*yBallPosition + yBallSetpoint;
    rollAngle = -K2*xSpeed - K1*xBallPosition + xBallSetpoint;

    //Serial.print("Ball y pos: ");
    //Serial.println(K2*xSpeed);    
    //Serial.println(K1*xBallPosition);

    
    setServoPosition(-pitchAngle, rollAngle, 0);

    // Store the last positions
    lastPosX = xBallPosition;
    lastPosY = yBallPosition;
  }
}

void setServoPosition(float pitchAngle, float rollAngle, float offset) {
  /*Serial.println(pitchAngle);
  Serial.println(rollAngle);
*/
  // Calculate the angle in radians
  pitchAngle = pitchAngle/360*2*PI;
  rollAngle = rollAngle/360*2*PI;

  // Calculate the changes for the servos
  float P1setpoint = (sqrt(3) * centerLength / 6.0) * sin(pitchAngle) * cos(rollAngle) + (centerLength / 2.0) * sin(rollAngle) + offset;
  float P2setpoint = sqrt(3) * centerLength / 6.0 * sin(pitchAngle) * cos(rollAngle) - centerLength / 2.0 * sin(rollAngle) + offset;
  float P3setpoint = -sqrt(3) * centerLength / 3.0 * sin(pitchAngle) * cos(rollAngle) + offset;

  // Set the setoints as a difference from the center pos
  int P1Setpoint = int(asin(P1setpoint*1.0 / armLength)/PI*180 + 90.0);
  int P2Setpoint = int(asin(P2setpoint*1.0 / armLength)/PI*180 + 88.0);
  int P3Setpoint = int(asin(P3setpoint*1.0 / armLength)/PI*180 + 94.0);

  
  // Constraint the setpont for the servo motor
  P1Setpoint = min(P1Setpoint, 110);
  P2Setpoint = min(P2Setpoint, 108);
  P3Setpoint = min(P3Setpoint, 114);

  P1Setpoint = max(P1Setpoint, 70);
  P2Setpoint = max(P2Setpoint, 68);
  P3Setpoint = max(P3Setpoint, 74);

  // Write the setpoints to the servos
  P1Servo.write(P1Setpoint);
  P2Servo.write(P2Setpoint);
  P3Servo.write(P3Setpoint);
  /*
  Serial.println(P1Setpoint);
  Serial.println(P2Setpoint);
  Serial.println(P3Setpoint);
*/
}


/**
   @brief Finds the center position of the largest object for the pixy blocks
   @param blocks Give the pixy blocks to be analysed
   @return xPosition long value for the center of the object
*/
void getPosition(uint16_t blocks, long *xPosition, long *yPosition) {
  int j;
  int largestObject = 0;
  int areaLargestObject = 0;

  for (j = 0; j < blocks; j++) {
    // Checks if the colour of the object is correct and if the position is in the correct area
    bool checkObjectPosition = (pixy.ccc.blocks[j].m_signature == 1) &&
                               (yAxis[0] < pixy.ccc.blocks[j].m_y) &&
                               (pixy.ccc.blocks[j].m_y < yAxis[1]) &&
                               (xAxis[0] < pixy.ccc.blocks[j].m_x) &&
                               (pixy.ccc.blocks[j].m_x < xAxis[1]);

    if (checkObjectPosition) {
      int areaObject = pixy.ccc.blocks[j].m_height * pixy.ccc.blocks[j].m_width;
      if (areaLargestObject >= areaObject) {
        areaLargestObject = areaObject;
        largestObject = j;
      }

    }
    // Check for the position of the object

    // Transform the objects coordinates to meters and print the result

    //if (i = 30) { pixy.blocks[largestObject].print(); }
  }
  *xPosition = pixy.ccc.blocks[largestObject].m_x;
  *yPosition = pixy.ccc.blocks[largestObject].m_y;

  //pixy.ccc.blocks[largestObject].print();
  //return xPosition, yPosition;
}


/**
   @brief Finds the center position of the largest object for the pixy blocks
   @param blocks Give the pixy blocks to be analysed
   @return xPosition long value for the center of the object
*/
void scalePosition(long *xPosition, long *yPosition, float *xPosScaled, float *yPosScaled) {

  float xPixel2Meter = plateDiameter/ (xAxis[1] - xAxis[0]);
  float yPixel2Meter = plateDiameter/-(yAxis[1] - yAxis[0]);
    /*
    Serial.println();

  Serial.print("meter per pixel x: ");
  Serial.println(xPixel2Meter);
  Serial.print("meter per pixel y: ");
  Serial.println(yPixel2Meter); */
  long center[2];

  
  center[0] = long((xAxis[1] - xAxis[0]) / 2 + xAxis[0]);
  center[1] = long((yAxis[1] - yAxis[0]) / 2 + yAxis[0]);
/*
  Serial.print("Center for x-axis: ");
  Serial.println(center[0]);
  Serial.print("Center for y-axis: ");
  Serial.println(center[1]);
*/
  *xPosScaled = (*xPosition - center[0]) * xPixel2Meter;
  *yPosScaled = (*yPosition - center[1]) * yPixel2Meter;
}

/* Sets a time in the future dependent on given argument

    @param timeout - the array containing the timeout period in the first index

    @return nextTimeout - time for when the timer is expired
*/
long startTimer (long timeout) {
  long nextTimeout = millis() + timeout;
  return nextTimeout;
}

/* Returns true if time is equal or higher than the previously set 'next timeout'

    @param timerArray - The array containing the next timeout value in the second index

    @return timerExpired - returns true if the timer is expired
*/
bool isTimerExpired(long timer) {
  // Check if the timer is expired by comparing the timeout value stored in the first array position.
  bool timerExpired;
  if (millis() >= timer) {
    timerExpired = true;
  } else {
    timerExpired =  false;
  }
  return timerExpired;
}
