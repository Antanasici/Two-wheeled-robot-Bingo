# Two wheeled self-balancing robot "Bingo"

## Introduction to the project...

A quick look at the range of mobile robots in existance reveals an enormous diversity in shape, form, and modes of mobility. However, one thing that most of them have in common is that they are passively balanced (i.e. their bodies are constantly in a state of stable equilibrium). While this is perfectly logical in most cases, there are certain applications, such as Segways and humanoid robots, that take advantage of an unstable-equilibrium, inverted pendulum design to enhance their capabilities. While their self-balancing mechanisms may increase the complexity of their design, the benefits, which include greater maneuverability and stability, outweigh the costs.

These systems use the same controllers that can be seen in servo-motors, air-conditioning units, and even thermostats. Of course rockets use significantly more complex controllers than air-conditioners, but the underlying principle is still the same: how to adjust the system in order to get as close to the desired outcome as possible. That is why building a self-balancing robot is so educational; you can use the same control methods over and over again for other projects.

To demonstrate the benefits of such a design, we built an upright, self-balancing, two-wheeled robot utilizing an IMU and a PID feedback control loop to maintain stability. While doing this we hope to get a good knowladge on PID control loops, so we can easily use them in future systems.

**PICTURE**
Figure 1: Our self balancing robot "Bingo". 

# How it was done...

**PICTURE**
Figure 2: Control loop

Our self-balancing robot design is essentially an inverted pendulum, which is a pendulum with its center of mass above the pivot point. Balancing an inverted pendulum is a challenge, because it is inherently unstable. The slightest disturbance from equilibrium position results in a force away from equilibrium that further destablizes the system. Therefore, keeping balance at an unstable equilibrium requires precise, low-latency control to instantly correct any errors in tilt the instant they happen.

To deal with this problem, we employed a PID controller that uses tilt feedback to control the torque of the motors and keep the robot balanced. A PID controller continuously measures a process variable (the tilt of the robot) and calculates an error value (angle from the vertical), which is the deviation of the process variable from some desired, ideal value (0 degrees from the vertical). The controller attempts to minimize this error over time by continuously adjusting a control variable (motor torque) according to the following equation, where u(t) is the control variable, e(t) is the current error in the process variable, and Kp, Ki, and Kd are coefficients that must be tuned to achieve the desired behavior of the controller:

**PICTURE**
Fundamental equation for a PID controller 

As seen from the equation the PID control variable depends on the current error (the proportional term), the rate of change of the error (the derivative term), and the long-term bias of the error (integral term), hence PID. This feedback loop, outlined in Figure 1 below, is the core of the robot's balancing behavior.

While PID control is a proven technique and is widely used for a variety of different applications, it relies on accurate sensor measurements in order to properly do its job. Raw sensor data always contains a significant amount of noise that could be detrimental to the functionality of the robot if not handled properly. This noise comes from a variety of different sources, and effects different sensors in different ways. Therefore, we will exploit this fact to devise a sensor fusuion algorithm that builds on the strengths of each of the individual sensors to make up for the weaknesses of thge others.

While we can derive tilt by using just an accelerometer, the accelerometer measurements are prone to error in the short term, due to horizontal acceleration of the robot. So we average the measurements over time (low pass filter) to get a more accurate long-term reading, but also one that has significant latency. On the other hand, integrating gyroscope measurements is a very good predictor of tilt in the short term, but has significant drift in the long term. Because the gyroscope is good in the short term and the accelerometer is good in the long term, we average the two together in a weighted average in what is called a complementary filter.

### Design Overview...

The electronics for this system can be divded into three main components - the microcontroller, the measurement subsystem, and the motor driver subsystem. For this project, we used a Arduino Uno microcontroller unit. Multiple times a second, the microcontroller reads raw data from the measurement subsystem and sends PWM signals to the motor driver to be converted to mechanical motion. The measurement subsystem consists of an Neuftech GY-521 MPU-6050 Module, a 6 degrees of freedom IMU that has an onboard three-axis accelerometer and a three-axis gyroscope. At a rate of 100Hz, the IMU measures the three-dimentional gravitational and rotational vectors and sends them to the DMP to be processed. When fresh IMU data is received, the DMP filters out sensor noise, and then fuses the data from the two sensors together to produce a single reading of tilt. Once the current tilt is known, the DMP calculates the error from the desired tilt, in this case the vertical, and then uses PID to control the PWM output to the motor driver subsystem. The motor driver subsystem consists of a MD25 motor driver connected to two 12V EMG-30 gear motors. The systems are powered using 12v Li-Po battery. We also added a Raspberry Pi v3 with a 5" HDMI LCD display unit to aid in debugging purposes, such as to display the current tilt, rotation rate, or the currently selected PID coefficients that can be tuned via serial monitors using same touchscreen.

We have also implamented a small system for wall following with an additional attachable block to the front of the robot.

**PICTURE*
Figure 3: Hardware circuit design 

### Raspberry Pi and camera

 One of our additional fetures is that our robot has a raspberry pi with a LCD touchsreen as the top platfrom, with camera attached to it.

We used the LCD touchscreen to make our lives esier and help with debugging, finding right PID coefficients and adjusting them if robot was improved in hardware level. We used a great library for the camera, and now we are able to view what camera is facing via PC/Smartphone or even the LCD on top of it. This provides us with amazing futures, recording videos, motion detection and also picture taking, which can be viewed later on from the same interface.

## Mechanical Design

### Frame

For building our frame at first we used wooden sheets about 2mm thick, but after some testing, we found out that our motors are to heavy for this, and they would bend the frame, we diecided to use acrylic glass sheets of 2mm and 4mm thicknes which provided us with suitable solution to remove vibration and bendings. We used 4 platforms to stack our hardware and supported them by M4 threaded rods, and secured platforms in required heigh by machine screw nuts. Motors were secured at slightly wider platfrom at the bottom, the battery was positioned at the 3rd platform from bottom. While the top platfrom were used for Raspberry Pi and Camera.

### EMG 30 Motors

The EMG30 (encoder, motor, gearbox 30:1) is a 12v motor fully equipped with encoders and a 30:1 reduction gearbox. It is ideal for small or medium robotic applications, providing cost effective drive and feedback for the user. It also includes a standard noise suppression capacitor across the motor windings.

Minimum Speed 1.5rpm
Maximum Speed 200rpm

Using the hub that came with them fit perfectly around the body of the motors, and had two holes that conveniently connected to the bottom platform of the robot with screws and nuts.

**PICTURE**
Figure 4: EMG-30 Measurements from datasheet

### Wheels

A 100mm diameter wheel with 5mm diameter hub for easy attachment to the EMG30, the wheel has a 26mm wide rubber tread which resulted in a better grip with surface/floor. Wheels were included in MD25 and EMG30 packs, also with the pack we got mounting bracket for easy motor fitting that are made from 2mm thick aluminium and are finished with blue enamel.

## Hardware Design 

**PICTURE**
Figure 5: Changes between the versions.

### Arduino Uno Rev3 Microcontroller
The Arduino Uno microcontroller serves as the brain of the robot. It reads the sensor data, filters and fuses it together, runs the PID controller and controlls the operations of the motors.

### Neuftech GY-521 MPU-6050 Module
We decided to use a combined accelerometer & gyroscope breakout module (the MPU-6050), which is slightly more expensive than a simple gyro, but should lead to a superior stabilisation performance. Plus it comes with it's own library.

We read these three values once every 10ms, and fuse them and derive an accurate and low-latency tilt measurement. This tilt measurment is what is fed into the PID controller to balance the robot.

In order to improve the stability of the robot, we took a few extra steps to calibrate the sensors and achieve more accurate measurements. First of all, when the robot is turned on, it should be lying on its side on a stable surface to allow it to calibrate its gyroscope. It does so by taking a large number of measurements while stable, averaging them, and subtracting the offset from any subsequent measurements.

### MD25 Motor Drivers

The MD25 is a robust I2C or serial, dual motor driver, designed for use with our EMG30 motors. We can drives two motors with independent or combined control. 12V is required to power the module. Motors can be commanded to turn by sent value. 

### Ultrasonic Distance Sensor HC-SR04

The HC-SR04 Ultrasonic Range Sensor uses non-contact ultrasound sonar to measure the distance to an object - they're great for any obstacle avoiding systems on Raspberry Pi robots or rovers! The HC-SR04 consists of two ultrasonic transmitters (basically speakers), a receiver, and a control circuit. The transmitters emit a high frequency ultrasonic sound, which bounce off any nearby solid objects, and the reciever listens for any return echo. That echo is then processed by the control circuit to calculate the time difference between the signal being transmitted and received. This time can subsequently be used, along with some clever math, to calculate the distance between the sensor and the reflecting object!

### Power
In order to power our robot, we needed a power source that could supply 12V and sufficient current to power 2 motors. We decided on a Turnigy LiPo Battery 11.1v with a capacity of 4500mAh, about 65 - 130 discharges. We also used a fuse and a switch to protect our motor drive and easy control.

## Software Design
### Bingo_Final.ino
Main file of our project, that contains almost all the systems used in the project. Starting by simple setup of the software and finishing with debbuging messages. We are using a MPU library to get the raw data from our MPU and then we use DMP library to remove all the noise and calculate tilt error. Then we use a PID loop to calculate the amount of PWM needed to compensate this error. All this function loops around once in 10ms so about 10 times a second. PID calculates the PWM needed to remove the tilt error and get robot back to its "original" setpoint. PWM is sent to motors to initializes the stabilisation of the robot. 

## Results
### Performance
Having LCD to change the PID coefficients was an enormous help, because it saved us an immeasureable amount of time while tuning the PID controller. By gradually tuning the PID coefficients, we were able to improve the stability of the system. After a lot of trial and error, we successfully got our robot to balance. If the gyroscope is properly calibrated, and the robot is placed gently in an upright position, it stays upright despite disturbances that would normally make it fall over. It is even able to carry a significant load and remain perfectly balanced.

We noticed that after a while of balancing it tends to leave his original "starting point" and move away from it, after that he is not able to maintain balance without moving. This error we plan to fix in the upcomng update of the robot.

Also we found out, that our system has a bug, which freezes our robot and takes away hes abillity to balance. After a lot of testing, we found out that this is not a hardware fault, but instead it is a clash happening between Wire (the I2c communication library) and MPU library. We do plan to reeaserch this more to fix this problem, but as a MPU library is not supported anymore we plan to write an algorithm to do the MPU data extraction ourselvs.

## Conclusion
### Evaluation
We came into this project expecting to build a two-wheeled robot that would balance itself with the help of an IMU. It took a good amount of work, and we encountered significant challenges, but we met our expectations and achieved our goal. After building the chassis, designing and testing the circuits, writing the software, and tuning the PID coefficients, we were able to sucessfully balance the robot on the two wheels, and even carry a load. But it is still not perfect - the few issues detailed above, including the minor wobble, the asymmetrical motor speeds, and the lack of encoders are small problems that can be fixed in a future update.
### Future Improvements

   * Fix the arduino freezing bug
   * Make it resistan to push and pull
   * Start using EMG 30 encoders to measure and control its speed. That will prevent it from approaching its maximum speed and falling over.
   * Implement manual remote steering of the robot, commanding it to move forward, backward, and rotate clockwise or counter-clockwise.
   * Adding light or color sensors at the bottom to make it follow a line on the ground.
   * Implament a control system via a controlle and web interface.
   * Create an interface to give graphs on the PWM, Tilt, Tilt error, and other properties, also abillity to changecoefficient from the same interface.
   * Improve the design to make space for additional hardware installation.
   * Implament wall following hardware in to the main body, and make the wall following function to work while balancing on two wheels.
   * Add smoke sensors, humidity sensors, Co2 sensors. All the data from the sensors needs to be displayed in the interface
   * Add a wireless communication that does not require an internet connection.
   * Use wall following to create a map of surroundings.
   
## Acknowledgements 

    Franco Raimondi - for helping with PID tuning.
    Michael Heeney - for providing the required parts.
    SLA Povilas Urbonas - for guidance on lasser cutting and experianced advice.
    Simon from Irelands - for showing a good example of a self balancing robot.
    Jeff Rowberg - the creator of I2cdev library (MPU-6050 and DMP librarys).
    And many great lecturers and tutors from Middelsex University - for answering many questions and helping to solve problems that we faced.
    All the great people of The Internet - for providing many tutorials and greate advice on PID and inverted pendulum.
## Meta
