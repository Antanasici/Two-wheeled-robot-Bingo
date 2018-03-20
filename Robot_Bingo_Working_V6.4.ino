/*--  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --
 * This is the version 6.4 of self balancing robot "Bingo".
 * Some parts of the code was writen by Simon B, for his own balancing robot. Same PID were used for our robot.
 * This project was build by Rimma Chepik & Antanas Icikovic.
 --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --  -*/
 
 /*-- --  Librarys that were included here  --  --*/
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <MD25IIC.h>

 /*-- --  I2C addresses  --  --*/
MPU6050 mpu;              
MD25IIC MyBoard;

 /*-- --  Define the pin-mapping  --  --*/
#define BTN_1 7                 // On/Off Button
#define BTN_2 4                 // Set Centre of Gravity Button
#define LED_1 2                 // Low-battery Warning LED
#define LED_2 10                // Current mode LED


// Max PWM parameters
#define MAX_TURN 30 // 128?


// MPU Control/Status
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
bool dmpReady = false;          // Set true if DMP init was successful
uint8_t devStatus;              // Return status after device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint16_t packetSize;            // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer


// Orientation/Motion
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
Quaternion q;                   // [w, x, y, z]       Quaternion Container
VectorFloat gravity;            // [x, y, z]            Gravity Vector
int16_t gyro[3];                // [x, y, z]            Gyro Vector
float ypr[3];                   // [yaw, pitch, roll]   Yaw/Pitch/Roll & gravity vector
float averagepitch[50];         // Used for averaging pitch value


// For PID Controller
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
//----------------------------------------------------
// I think this are good PID:
// P = 0.75 | 0.9 | 0.8 | 0.65 | 0.9
// I = 0. 05| 0.1 | 0   | 0.13 | 0.05
// D = 0.35 | 0.4 | 0   | 0.25 | 9
//----------------------------------------------------
float Kp = 0.95;                  // (P)roportional Tuning Parameter
float Ki = 0.05;                  // (I)ntegral Tuning Parameter        
float Kd = 9;                 // (D)erivative Tuning Parameter       
float lastpitch;                  // Keeps track of error over time
float iTerm;                      // Used to accumulate error (integral)
// Pitch on the total flat right - Pitch: 82.136 KAMERA MINUSAS!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Ptch on the total flat left - Pitch: -79.253 
// Look so arduino port would be facing the same direction to find where is left where is right 
// Target Pitch: 8.612, Target Yaw: 2.219Target Pitch: 8.442, Target Yaw: 1.771
//Target Pitch: 8.676, Target Yaw: -1.611Target Pitch: 8.404, Target Yaw: 1.781
//Target Pitch: 8.469, Target Yaw: -5.172Target Pitch: 8.655, Target Yaw: -1.623
//Target Pitch: 8.670, Target Yaw: -1.650
// Good Angle I found are:
// = 9.606 | 10.379 | 10.874 | 10.834 | new - 2.065, 4.329, 
float targetAngle = 4.00;        // Can be adjusted according to centre of gravity 

// You can Turn off YAW control, by setting
// the Tp and Td constants below to 0.
float Tp = 0.6;                     // Yaw Proportional Tuning Parameter (0.6)
float Td = 0.1;                     // Yaw Derivative Tuning Parameter (0.1)
float targetYaw = 0;              // Used to maintain the robot's yaw
float lastYawError = 0;           
float PIDGain = 0;                // Used for soft start (prevent jerking at initiation)

// Runtime variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int modeSelect = 1;             
bool initialised = true;       

char inchar = 0;                // Hold any incoming characters
float angular_rate = 0;         // Used to make sure rate is ~0 when balance mode is initiated

bool newCalibration = false;  // If set TRUE, the target angles are recalibrated


// Variables used for timing control
// Aim is 10ms per cycle (100Hz)
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define STD_LOOP_TIME 10

unsigned long loopStartTime = 0;
unsigned long lastTime;             // Time since PID was called last (should be ~10ms)

// 0 = Off, 1 = On
int modes = 1;



// ------------------------------------------------------------------
//                INITIAL SETUP
// ------------------------------------------------------------------

void setup() {

    Wire.begin();

    // Initialize serial communication for debugging
    Serial.begin(115200);
  
   // Configure LED for output
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);

    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, LOW);

    // Set as input, internal pullup for buttons
    pinMode(BTN_1, INPUT_PULLUP);
    pinMode(BTN_2, INPUT_PULLUP);
    // MD25
    MyBoard.enableTimeOut(true);    // Stop motor if no command sent  // within a 2s window
    MyBoard.enableController(true); // Enable On-board speed controller
    MyBoard.setMode(1);
    MyBoard.resetEncoders();        // Reset (both) encoders to 0000
    MyBoard.setAcceleration(1);   // Increase acceleration (default = 5)
    MyBoard.setMotor1Speed(0);   // The left motor is initially stoped
    MyBoard.setMotor2Speed(0);   // The right motor is initially stoped
    // Initialize MPU6050
    mpu.initialize();
    delay(10);
    Serial.println("T:");
    delay(100);
    printf("Starting");
    Serial.println(mpu.testConnection() ? "> MPU6050 connection successful" : "> MPU6050 connection failed");
    Serial.println("Initialising DMP");
    devStatus = mpu.dmpInitialize();

    /* * * * * * * * * * * * * * * * * * * *
     * IMPORTANT!
     * Supply your own MPU6050 offsets here
     * Otherwise robot will not balance properly.
     * * * * * * * * * * * * * * * * * * * */
    mpu.setXGyroOffset(108);                     // 93    | 125
    mpu.setYGyroOffset(-42);                    // -15     | 24
    mpu.setZGyroOffset(54);                     // 30     | 65
    mpu.setXAccelOffset(-2355);                 // -2500  | -1340 V
    mpu.setYAccelOffset(27);                  // 1783   | 4047
    mpu.setZAccelOffset(1368);                   // 877   | 1580

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        Serial.println("Enabling DMP");
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println("DMP Ready! Let's Proceed.");
        Serial.println("Robot is now ready to balance. Hold the robot steady");
        Serial.println("in a vertical position, and the motors should start.");
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

    } else {
    // In case of an error with the DMP
        if(devStatus == 1) Serial.println("> Initial Memory Load Failed");
        else if (devStatus == 2) Serial.println("> DMP Configuration Updates Failed");
    }

}



// -------------------------------------------------------------------
//       PID CONTROLLER
// -------------------------------------------------------------------

int PID(float pitch) {            

    // Calculate time since last time PID was called (~10ms)
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    unsigned long thisTime = millis();
    float timeChange = float(thisTime - lastTime);

    // Calculate Error
    float error = targetAngle - pitch;


    // Calculate our PID terms
    // PID values are multiplied/divided by 10 in order to allow the
    // constants to be numbers between 0-10.
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    float pTerm = Kp * error * 10;
    iTerm += Ki * error * timeChange / 10;  
    float dTerm = Kd * (pitch - lastpitch) / timeChange * 100; 
  
  if (Ki == 0) iTerm = 0;
    lastpitch = pitch;
    lastTime = thisTime;


    // Obtain PID output value
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    float PIDValue = pTerm + iTerm - dTerm;

    // Set a minimum speed (motors will not move below this - can help to reduce latency)
    //if(PIDValue > 0) PIDValue = PIDValue + 10;
    //if(PIDValue < 0) PIDValue = PIDValue - 10;

  // Limit PID value to maximum PWM values
    if (PIDValue > 127) PIDValue = 127;
    else if (PIDValue < -128) PIDValue = -128; 

    return int(PIDValue);

}



// -------------------------------------------------------------------
//       YAW CONTROLLER
// -------------------------------------------------------------------

int yawPD(int yawError) {            


    // Calculate our PD terms
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    float pTerm = Tp * yawError;
    float dTerm = Kd * (yawError - lastYawError) / 10; 
  
    lastYawError = yawError;

    // Obtain PD output value
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    int yawPDvalue = int(-pTerm + dTerm);

  // Limit PD value to maximum
    if (yawPDvalue > MAX_TURN) yawPDvalue = MAX_TURN;
    else if (yawPDvalue < -MAX_TURN) yawPDvalue = -MAX_TURN; // - on both before MAX_TURN!!!!

    //Serial.print("Error: ");
    //Serial.print(yawError);
    //Serial.print(" - PD: ");
    //Serial.println(yawPDvalue);
    return yawPDvalue;

}



// -------------------------------------------------------------------
//        MOVEMENT CONTROLLER
// -------------------------------------------------------------------
// This function calculate the PWM output required to keep the robot 
// balanced, to move it back and forth, and to control the yaw.

void MoveControl(int pidValue, float yaw){
  
    // Set both motors to this speed
    int left_PWM = pidValue;
    int right_PWM = pidValue;

    //Serial.println(pidValue);


    /* YAW CONTROLLER */

    // Check if turning left or right is faster
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    int leftTurn, rightTurn;

    float newYaw = targetYaw;

    if((yaw > 0) && (newYaw < 0)){
        rightTurn = yaw + abs(newYaw);
        leftTurn = (180 - yaw) + (180 - abs(newYaw));

    } else if ((yaw < 0) && (newYaw > 0)){
        rightTurn = (180 - abs(yaw)) + (180 - newYaw);
        leftTurn = abs(yaw) + newYaw;

    } else if (((yaw > 0) && (newYaw > 0)) || ((yaw < 0) && (newYaw < 0))){
        rightTurn = newYaw - yaw;

        if (rightTurn > 0){
            leftTurn = rightTurn;
            rightTurn = 360 - leftTurn;
        } else if (rightTurn < 0){
            rightTurn = abs(rightTurn);
            leftTurn = 360 - abs(rightTurn);
        } else if (rightTurn == 0){
            rightTurn = leftTurn = 0;
        }
    }

    // Apply yaw PD controller to motor output
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    if ((leftTurn == 0) && (rightTurn == 0)){
        // Do nothing
    } else if (leftTurn <= rightTurn){
      leftTurn = yawPD(leftTurn);
        left_PWM = left_PWM - leftTurn;
        right_PWM = right_PWM + leftTurn;

    } else if (rightTurn < leftTurn){
        rightTurn = yawPD(rightTurn);
        left_PWM = left_PWM + rightTurn;
        right_PWM = right_PWM - rightTurn;
        
    }


    // Limits PID to max motor speed
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
    if (left_PWM > 127) left_PWM = 127;
    else if (left_PWM < -127) left_PWM = -127; 
    if (right_PWM > 127) right_PWM = 127;
    else if (right_PWM < -127) right_PWM = -127; 

    // Send command to left motor
    if (left_PWM >= 0) MyBoard.setMotor1Speed((int(right_PWM)));     // '0' = Left-motor, '1' = Right-motor
    else MyBoard.setMotor1Speed((int(right_PWM)));
  // Send command to right motor
    if (right_PWM >= 0) MyBoard.setMotor2Speed((int(left_PWM)));   // '0' = Forward, '1' = Backward
    else MyBoard.setMotor2Speed((int(left_PWM)));           // Error here, shouldn't be -1 

}


// -------------------------------------------------------------------
//       READ INPUT FROM SERIAL
// -------------------------------------------------------------------

void readSerial() {

    // Initiate all of the variables
    // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  int changestate = 0;    // Which action needs to be taken?
  int no_before = 0;      // Numbers before decimal point
  int no_after = 0;     // Numbers after decimal point
  bool minus = false;     // See if number is negative
  inchar = Serial.read();   // Read incoming data

    if (inchar == 'P') changestate = 1;
    else if (inchar == 'I') changestate = 2;
    else if (inchar == 'D') changestate = 3;

    // Tell robot to calibrate the Centre of Gravity
    else if (inchar == 'G') calibrateTargets();


    // Records all of the incoming data (format: 00.000)
    // And converts the chars into a float number
    if (changestate > 0){
        if (Serial.available() > 0){

            // Is the number negative?
            inchar = Serial.read();
            if(inchar == '-'){
                minus = true;
                inchar = Serial.read();
            }
            no_before = inchar - '0';

            if (Serial.available() > 0){
                inchar = Serial.read();

                if (inchar != '.'){
                    no_before = (no_before * 10) + (inchar - '0');

                    if (Serial.available() > 0){
                        inchar = Serial.read();
                    }
                }

                if (inchar == '.'){
                    inchar = Serial.read();
                    if (inchar != '0'){
                        no_after = (inchar - '0') * 100;
                    }

                    if (Serial.available() > 0){
                        inchar = Serial.read();
                        if (inchar != '0'){
                            no_after = no_after + ((inchar - '0') * 10);
                        }

                        if (Serial.available() > 0){
                            inchar = Serial.read();
                            if (inchar != '0'){
                                no_after = no_after + (inchar - '0');
                            }
                        }
                    }
                }
            }

            // Combine the chars into a single float
            float answer = float(no_after) / 1000;
            answer = answer + no_before;
            if (minus) answer = answer * -1;

            // Update the PID constants
            if (changestate == 1){
                Kp = answer;
                Serial.print("P - ");
            } else if (changestate == 2){
                Ki = answer;
                Serial.print("I - ");
            } else if (changestate == 3){ 
                Kd = answer;
                Serial.print("D - ");
            }
            Serial.print("Constant Set: ");
            Serial.println(answer, 3);

        } else {
            changestate = 0;
        }
    }
}



// -------------------------------------------------------------------
//       RECALIBRATE TARGET VALUES
// -------------------------------------------------------------------
// Takes a number of readings and gets new values for the target angles.
// Robot must be held upright while this process is being completed.

void calibrateTargets(){

  targetAngle = 0;
  targetYaw = 0;
  
    for(int calibrator = 0; calibrator < 50; calibrator++){
  
    accelgyroData();
    targetAngle += pitch();
    targetYaw += yaw();
    delay(10);
  }
  
  // Set our new value for Pitch and Yaw
  targetAngle = targetAngle / 50;
  targetYaw = targetYaw / 50;
  Serial.print("Target Pitch: ");
  Serial.print(targetAngle, 3);
  Serial.print(", Target Yaw: ");
  Serial.print(targetYaw, 3);

  newCalibration = false;
}



// -------------------------------------------------------------------
//       GET PITCH AND YAW VALUES
// -------------------------------------------------------------------
// This simply converts the values from the accel-gyro arrays into degrees.

float pitch(){
  return (ypr[1] * 180/M_PI);
}

float yaw(){
  return (ypr[0] * 180/M_PI);
}

float angRate(){
  return -((float)gyro[1]/131.0);
}



// -------------------------------------------------------------------
//       GET ACCEL_GYRO DATA
// -------------------------------------------------------------------

void accelgyroData(){

    // Reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println("Warning - FIFO Overflowing!");

    // otherwise, check for DMP data ready interrupt (this should happen exactly once per loop: 100Hz)
    } else if (mpuIntStatus & 0x02) {
        // Wait for correct available data length, should be less than 1-2ms, if any!
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get sensor data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.resetFIFO();

        //Serial.print(ypr[1]);
        //Serial.print(" - ");
        //Serial.println(ypr[0]);
    }
}



// -------------------------------------------------------------------
//       MAIN PROGRAM LOOP
// -------------------------------------------------------------------

void loop() {
  //Serial.println(pTerm);
  //Serial.println(iTerm);
  //Serial.println(dTerm);

  // If the "SET" button is pressed
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (digitalRead(BTN_2) == LOW){

    digitalWrite(LED_1, HIGH);
      calibrateTargets();

      lastpitch = 0;
      iTerm = 0;

      Serial.println("> Setting new centre of gravity <");

    delay(250);
      mpu.resetFIFO();
      digitalWrite(LED_1, LOW);
  }


  // If the "POWER" button is pressed
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  
  /*if (digitalRead(BTN_1) == LOW){
      if (modeSelect == 1){
          Serial.println("> Turning off balancing system <");
          initialised = false;
          modeSelect = 1;
          MyBoard.setMotor1Speed(0);
          MyBoard.setMotor2Speed(0);// Stop both motors from moving
          digitalWrite(LED_2, LOW);
      } else if (modeSelect == 0){
          Serial.println("> Turning on balancing system <");
          initialised = false;
          modeSelect = 1;
          digitalWrite(LED_2, HIGH);
      }
      delay(500);
      mpu.resetFIFO();
  } */
    
  // Gather data from MPU6050
  accelgyroData();
    
  // If the Balance System is turned on:
  if (modeSelect == 1){
        
    if (!initialised){

          // Wait until robot is vertical and angular rate is almost zero:
          if ((pitch() < targetAngle+0.1) && (pitch() > targetAngle-0.1) && (abs(angRate()) < 0.3)){
              Serial.println(">>>> Balancing System Active <<<<");
              initialised = true;
              lastpitch = pitch();
              iTerm = 0;
          }
  
      // Otherwise, run the PID controller
    } else {

      // Stop the system if it has fallen over:
      if ((pitch() < -300) || (pitch() > 300)){
          
        // Stop the motors
        MyBoard.setMotor1Speed(0);
        MyBoard.setMotor2Speed(0);
        // Reset runtime variables
        lastpitch = 0;
        iTerm = 0;
        initialised = false;
        Serial.println(">>>> Balancing System Stopped <<<<");

      } else {
        // A bit of function-ception happening here:
        Serial.println(pitch());
        MoveControl(PID(pitch()), yaw());
      }
    }
  }

    if (Serial.available() > 0){    // If new PID values are being sent by the interface
        readSerial();               // Run the read serial method
    }

    // Call the timing function
    // Very important to keep the response time consistent!
    timekeeper();
}



// -------------------------------------------------------------------
//        TIME KEEPER
// -------------------------------------------------------------------

void timekeeper() {

    // Calculate time since loop began
    float timeChange = millis() - loopStartTime;

    // If the required loop time has not been reached, please wait!

    if (timeChange < STD_LOOP_TIME) {
        delay(STD_LOOP_TIME - timeChange);
    } 


    // Update loop timer variables
    loopStartTime = millis();   
}

