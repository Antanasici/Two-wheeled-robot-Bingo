/*
  HC-SR04 Ping distance sensor
  VCC to Arduino 5V
  GND to Arduino GND
  Echo to Arduino pin 13
  Trig to Arduino pin 12
  More info at: http://goo.gl/kJ8Gl
  Original code improvements to the Ping sketch sourced from Trollmaker.com
  Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
  Modified by Tolson Winters (Aug 27, 2014) for simplified serial monitor reading.
  Modified by Antanas & Rimma (April 4, 2017) for "Bingo" project.
*/

/***************************************************************
  // MOTOR CONTROL WITH MD-25 DRIVER BOARD
  //**************************************************************/
//http://www.seattlerobotics.org/encoder/200108/using_a_pid.html
#include <Wire.h>
#include <MD25IIC.h>
//ultrasonic sensors
//front sensors
#define trigPin A0
#define echoPin A1



//----------------------------------------
// Data
//----------------------------------------

#define MODE                0x34

//----------------------------------------
//
// MD25IIC METHODS:
//    -----------------------   ------------------------
//   |       MOTOR          |  | BOARD                 |
// --------------------------  |-----------------------|
// 1 | setMotor1Speed(byte) |  | getAddress()          |
//   | getMotor1Encoder()   |  | setMode(byte)         |
//   | getLmotorCurrent()   |  | getBattery()          |
// -------------------------|  | enableTimeOut(T/F)    |
// 2 | setMotor2Speed(byte) |  | enableController(T/F) |
//   | getRmotorEncoder()   |  | resetEncoders()       |
//   | getRmotorCurrent()   |  | setAcceleration(byte) |
// -------------------------|  |-----------------------|

MD25IIC MyBoard;
/***************************************************************
  // Setup function
  //**************************************************************/


/***************************************************************
  // Return angle (between 0 and 359º)
  //**************************************************************/

/***************************************************************
  // Setup function
  //**************************************************************/

#define trigPin A0
#define echoPin A1


void setup() {
  Serial.begin (115200);
  MyBoard.enableTimeOut(true);    // Stop motor if no command sent  // within a 2s window
  MyBoard.enableController(true); // Enable On-board speed controller
  MyBoard.setMode(0);
  MyBoard.resetEncoders();        // Reset (both) encoders to 0000
  MyBoard.setAcceleration(10);   // Increase acceleration (default = 5)
  MyBoard.setMotor1Speed(128);   // The left motor is initially stoped
  MyBoard.setMotor2Speed(128);   // The right motor is initially stoped


  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);




}


void loop() {
  follow_wall();

}


void follow_wall() {
  //motor1 left speed 140 goin forward
  //motor2 rigg
  MyBoard.setMotor1Speed(135); // 152  150
  MyBoard.setMotor2Speed(135);  // 148  150
  Serial.print(sensor());
  Serial.println(" cm");

  do {
    Serial.print(sensor());
    Serial.println(" cm");
    //  MyBoard.setMotor1Speed(150); // 152
    //  MyBoard.setMotor2Speed(150);  // 148



    ///
    if (sensor() < 25 )

    {

      Serial.println(sensor());
      Serial.println(" Turning left");
      //  MyBoard.setMotor1Speed(130);  // 150  //152 150
      MyBoard.setMotor2Speed(135);   // 151 160
      //  print_encoders();

    }
    else if (sensor() > 30)// ( diff < 0 )
    {

      Serial.println(sensor());
      Serial.println(" Turning right");
      MyBoard.setMotor1Speed(135);   //151 //  152
      ///  MyBoard.setMotor2Speed(130);   //149
      //  print_encoders();
    }

    else if (sensor() > 25 &&  sensor() < 30)// ( diff < 0 )
    {

      Serial.println(sensor());
      Serial.println(" go straight");
      MyBoard.setMotor1Speed(135);   //151 //  152
      MyBoard.setMotor2Speed(135);   //149
      //  print_encoders();
    }

    else
    {
      //   Serial.print(diff);
      Serial.println(sensor());
      Serial.println(" STOP");
      // Serial.println(" Going straight");
      MyBoard.setMotor1Speed(128); // 152
      MyBoard.setMotor2Speed(128);  // 148
      //   print_encoders();
    }



  }
  while (MyBoard.getMotor2Encoder() < 100000);
  stop_motors();
  MyBoard.resetEncoders();

}




int sensor() {
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}

void follow_wall_using_sensors( int obs_distance ) {

  MyBoard.setMotor1Speed(150); // 152
  MyBoard.setMotor2Speed(150);  // 148

  if (sensor() < obs_distance)

  {

    Serial.println(sensor());
    Serial.println(" Turning Right");
    MyBoard.setMotor1Speed(150);  // 150
    MyBoard.setMotor2Speed(152);   // 151


  }
  else if (sensor() > obs_distance)// ( diff < 0 )
  {

    Serial.println(sensor());
    Serial.println(" Turning Left");
    MyBoard.setMotor1Speed(152);   //151
    MyBoard.setMotor2Speed(150);   //149

  }
  else
  {
    //   Serial.print(diff);
    Serial.println(sensor());
    Serial.println(" GOING STRAIGHT");
    // Serial.println(" Going straight");
    MyBoard.setMotor1Speed(150); // 152
    MyBoard.setMotor2Speed(150);  // 148

  }


  //while( 1==1);

  // while (MyBoard.getMotor1Encoder() <= encoder_counts);
  //  stop_motors();
  //  print_encoders();
  // MyBoard.resetEncoders();
}




void stop_motors() {
  MyBoard.setMotor1Speed(128);
  MyBoard.setMotor2Speed(128);
}



