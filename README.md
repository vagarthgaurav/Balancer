# Introduction 

A self balancing robot using 2 Nema 17 stepper motors and an MPU6050. It uses a PID controller for stabilization using the pitch angle measured by the IMU. 

# Hardware 

1. 2 Nema 17 stepper motors.
2. 2 TMC2208 stepper motor drivers.
3. ESP8266 board.
4. MPU6050 interntial measurement unit. 
5. 2S lipo. 
6. 2 Pololu style 80x10mm wheels - [model on printables](https://www.printables.com/model/1156620-80x10mm-wheel-with-5mm-d-shaft).


# Future 

## Center of mass problem 
Currently the robots center of mass is not aligned with the center line of the robot and this causes it to tilt forwards. This will be fixed in the future by adding another PID controller for the velocity of the motors, meaured using a magnetic encoder.

## PID tuning
The current PID values are not optimal and further tuning is required.