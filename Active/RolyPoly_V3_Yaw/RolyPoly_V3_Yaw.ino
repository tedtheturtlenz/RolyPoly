
#include <PID_v1_bc.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define PWMA 5
#define AIN2 21
#define AIN1 20
#define BIN1 19
#define BIN2 18
#define PWMB 6
#define STDBY 10
#define OUTPUT_READABLE_YAWPITCHROLL
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


int dataIn[7]{ 0, 125, 125, 125, 0, 0, 0 };
int in_byte = 0;
int array_index = 0;
int Xpos = 10;
int Ypos = 0;
int button1 = 0;
int phoneYaw = 0;
double Akp = 2;
double Aki = 0;
double Akd = 0;
double Pkp = 2;
double Pki = 0;
double Pkd = 0;
double degAngle = 30;
double realAngle = 0;
double altDegAngle = 0;
double altRealAngle = 0;
double motorSpeed = 20;
double yaw = 0;

double prevError = 0;
double errorSum = 0;
double desiredPitch = 180;
double realPitch = 0;
double errorPitchSum = 0;
double prevPitchError = 0;
double pitchMotorSpeed = 0;
double pitchBaseWheelSpeed = 0;
double RWheelSpeed = 0;
double LWheelSpeed = 0;
double baseWheelSpeed = 20;

int loopCounter = 0;


//MPU Stuff
//|============================================================|//
#define INTERRUPT_PIN 7  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  // put your setup code here, to run once:
  MCUCR = (1 << JTD);
  MCUCR = (1 << JTD);
  
  //Start Serials for port and BT
  //Serial.begin(9600);
  Serial.begin(9600);
 
  Serial1.begin(9600);
   delay(5000);
  
  while(!Serial1.available())
  {
    Serial.println("Waiting for BT Serial");
    delay(1000);
    
  }
  
  //Set PID to automatic not manual
  //anglePID.SetMode(AUTOMATIC);
  //Motor Stuff
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);

  // join I2C bus (I2Cdev library doesn't do this automatically)

  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties


  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)

  //while (!Serial)
  //;  // wait for Leonardo enumeration, others continue immediately


  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));



  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(139);
  mpu.setYGyroOffset(-56);
  mpu.setZGyroOffset(-1);
  mpu.setZAccelOffset(1726);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    //Serial.println(F(")"));
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //Start timer for MPU and integrating
  long startTime = millis();
  if (Serial1.available() > 0) {
  
    in_byte = Serial1.read();
    
    if (in_byte == (255)) {
      array_index = 0;
      Serial.println("We got to changing array index");
    }
    dataIn[array_index] = in_byte;
    //Serial.println("We got to updating array");
    array_index = array_index + 1;
  }
  else{
    Serial.println("Bluetooth Serial not available");
    //Serial.println(Xpos);
  }
  Xpos = dataIn[2];  //The X position of the joystick
  Ypos = dataIn[3];  //The Y position of the joystick
  //phoneYaw = 360 * dataIn[1] / 250;
  
  //Serial.println(phoneYaw);
  
  
  //The Top Left of the box is 0, 0. So, if we subtract 255/2 off of each value, we get the position w/ reference to the center of the box.
  double trigXpos = Xpos - 127;
  double trigYpos = -(Ypos - 127);

  //For example, if we are in the bottom right of the box, Xpos = 255, Ypos = 255, trigx = 128, trigy = 128
  //Use trig to find the angle between the origin and where the joystick is
  double radAngle = atan(trigXpos / trigYpos);
  //convert from radians to degrees
  degAngle = 180 * radAngle / 3.14;
  //This deals with tan not really working well for a whole rotation. Change to having 180 deg when robot pointing forward.
  if ((trigXpos <= 0) && (trigYpos >= 0)) {
    degAngle = degAngle + 360;
  } else if ((trigXpos <= 0) && (trigYpos <= 0)) {
    degAngle = degAngle + 180;
  }
  if ((trigXpos >= 0) && (trigYpos < 0)) {
    degAngle = degAngle + 180;
  }

  if (degAngle > 180) {
    degAngle = (degAngle - 360);
  }
  //And the Magnitude is simply a triangle
  double mag = sqrt(sq(trigXpos) + sq(trigYpos));
  baseWheelSpeed = mag;
  /*
Serial.print("The Angle of direction is ");
Serial.println(degAngle);
Serial.print("The magnitude is ");
Serial.println(mag);
Serial.print(trigXpos);
Serial.print(" ");
Serial.println(trigYpos);
*/
  //Serial.print("The magnitude is ");
  //Serial.println(mag);

  //Getting the actual angle of Roly
  //Insert MPU6050 stuff


if (!dmpReady) return;

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet



#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    //Serial.println(dataIn[2]);
    //Serial.println(dataIn[3]);
#endif
  }
  yaw = ypr[0] * 180 / M_PI;


  if (yaw > 360) {
    realAngle = yaw - 360;
  }


  else {
    realAngle = yaw;
  }
  //Do some phase shifting to get proper angles for PID centered at 180
  altRealAngle = realAngle + 180;
  altDegAngle = degAngle + 180;


  //get the real angles within 360
  if (degAngle > 360) {
    degAngle = degAngle - 360;
  }

  if (realAngle > 360) {
    realAngle = realAngle - 360;
  }
  //Get the alt angles to within 360 too

  if (altDegAngle > 360) {
    altDegAngle = altDegAngle - 360;
  }

  if (altRealAngle > 360) {
    altRealAngle = altRealAngle - 360;
  }
//get the real angles within 360
  if (degAngle <= 0) {
    degAngle = degAngle + 360;
  }

  if (realAngle <= 0) {
    realAngle = realAngle + 360;
  }
  //Get the alt angles to within 360 too

  if (altDegAngle <= 0) {
    altDegAngle = altDegAngle + 360;
  }

  if (altRealAngle <= 0) {
    altRealAngle = altRealAngle + 360;
  }


  double realDifference = (abs(realAngle - degAngle));
  double altDifference = (abs(altRealAngle - altDegAngle));

  if (altDifference < realDifference) {
    realAngle = altRealAngle;
    degAngle = altDegAngle;
  }
  
  
  

if(loopCounter < 100)
  {
    loopCounter = loopCounter + 1;
  }

  else
  {
    
   Serial.print(altDifference);
  Serial.print(" alt diff\t\t");
  Serial.print(realDifference);
  Serial.print(" real diff\n");
  Serial.print(altRealAngle);
  Serial.print(" alt angle\t\t");
  Serial.print(realAngle);
  Serial.print(" real angle\n");
  Serial.print(altDegAngle);
  Serial.print(" alt deg angle\t\t");
  Serial.print(degAngle);
  Serial.print(" real deg angle\n");
  Serial.print("YPos and Xpos ");
  Serial.print(Ypos);
  Serial.print(" ");
  Serial.println(Xpos);
  
    loopCounter = 0;
  }



  //======= Yaw PID ========
  double errorAngle = degAngle - realAngle;
  errorSum = errorSum + errorAngle;
  //DO some PID stuff
  //Change this back to pid library eventually.
  motorSpeed = (Akp * errorAngle) + (Aki * errorSum) + (Akd * (errorAngle - prevError));

  //======= Pitch PID =======
  realPitch = (ypr[2] * 180 / M_PI) + 180;
  double errorPitch = desiredPitch - realPitch;
  errorPitchSum = errorPitchSum + errorPitch;
  //DO some PID stuff
  //Change this back to pid library eventually.
  pitchMotorSpeed = (Pkp * errorPitch) + (Pki * errorPitchSum) + (Pkd * (errorPitch - prevPitchError));



  //Set the motor speeds, currently not trying to drive anywhere, eventually add in some motor speed common to both that is related to magnitude.
  //COnstrained to +-127 so that there can be forward and back
  RWheelSpeed = constrain(baseWheelSpeed - motorSpeed, -127, 127);
  LWheelSpeed = constrain(baseWheelSpeed + motorSpeed, -127, 127);
  //Now change motor speeds to the updated ones
  drive(RWheelSpeed, LWheelSpeed, mag);


  //Serial.print("Right Wheel Speed ");
  //Serial.print(RWheelSpeed);
  //Serial.print("\t \t Left Wheel Speed ");
  //Serial.print(LWheelSpeed);
  /*
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
  Serial.print("\t\t PID Speed: ");
  Serial.print(motorSpeed);
  Serial.print("\t\t Deg Angle: ");
  Serial.print(degAngle);
  Serial.print("\t\t Actual Angle: ");
  Serial.print(realAngle);
  Serial.print("\t\t Actual Pitch: ");
  Serial.print(realPitch);
  Serial.print("\r\n");
  */
  delay(10);
  //Update previous error
  prevError = errorAngle;
  prevPitchError = errorPitch;
}

void drive(double rightWheelSpeed, double leftWheelSpeed, double magnitude) {
  if (magnitude < 20) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  } else {
    if (rightWheelSpeed > 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);

      analogWrite(PWMA, int(rightWheelSpeed));

    } else if (rightWheelSpeed < 0) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      int rws = int(rightWheelSpeed * -1);
      //Serial.println(rws);
      analogWrite(PWMA, rws);
    }


    if (leftWheelSpeed > 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMB, int(leftWheelSpeed));
    } else if (leftWheelSpeed < 0) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      int lws = int(leftWheelSpeed * -1);
      analogWrite(PWMB, lws);
    }
  }
}
