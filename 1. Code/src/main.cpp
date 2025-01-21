#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <AccelStepper.h>
#include "ArduPID.h"

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector

float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define INTERRUPT_PIN 15 // use pin 15 on ESP8266

AccelStepper left(1, 13, 0);  //(Driver, Step, Dir)
AccelStepper right(1, 14, 2); //(Driver, Step, Dir)

long previousMillis = millis(); // Contador do tempo para ler MPU
long interval = 5;

// PID configuration
ArduPID balanceController;

double setpoint = 0;
double input;
double output;
double p = 2000;
double i = 0.5;
double d = 50;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady()
{
    mpuInterrupt = true;
}

void mpu_setup()
{

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(61);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(-9);
    mpu.setXAccelOffset(-663);
    mpu.setYAccelOffset(272);
    mpu.setZAccelOffset(1065);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setup(void)
{
    Serial.begin(115200);

    left.setMaxSpeed(10000);
    left.setAcceleration(8000);

    right.setMaxSpeed(10000);
    right.setAcceleration(8000);

    balanceController.begin(&input, &output, &setpoint, p, i, d);
    balanceController.setOutputLimits(-10000, 10000);
    balanceController.setBias(0);
    balanceController.setSampleTime(5);

    balanceController.start();

    mpu_setup();
}

double getPitch()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return 0;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize)
        return 0;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Serial.print(ypr[1] * 180 / M_PI);

        return (double)ypr[1] * 180 / M_PI;
    }
    return 0;
}

void loop(void)
{

    previousMillis = millis();

    input = getPitch();

    balanceController.compute();
    // balanceController.debug(&Serial, "balanceController", PRINT_INPUT | PRINT_OUTPUT);
    Serial.print(input);
    Serial.print("\t");
    Serial.println(output);

    while ((millis() - previousMillis) < interval)
    {

        left.setSpeed(-output);
        right.setSpeed(output);
        left.runSpeed();
        right.runSpeed();
    }
}
