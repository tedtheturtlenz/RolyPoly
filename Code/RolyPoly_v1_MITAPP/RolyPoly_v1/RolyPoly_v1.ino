
#include <PID_v1_bc.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define PWMA 5
#define AIN2 21
#define AIN1 20
#define BIN1 19
#define BIN2 18
#define PWMB 6
#define STDBY 10


int dataIn[4]{ 0, 0, 0, 0 };
int in_byte = 0;
int array_index = 0;
int Xpos = 0;
int Ypos = 0;
int button1 = 0;
int Akp = 2;
int Aki = 0;
int Akd = 0;
double degAngle = 0;
double realAngle = 0;
double motorSpeed = 0;
PID anglePID(&realAngle, &motorSpeed, &degAngle, Akp, Aki, Akd, DIRECT);
double prevError = 0;
double errorSum = 0;
double RWheelSpeed = 0;
double LWheelSpeed = 0;
double baseWheelSpeed = 0;


Adafruit_MPU6050 mpu;
float yaw = 0;
float pitch = 0;

void setup() {
  // put your setup code here, to run once:
  MCUCR = (1 << JTD);
MCUCR = (1 << JTD);
  //Start Serials for port and BT
  Serial.begin(9600);
  Serial1.begin(9600);
  //Set PID to automatic not manual
  anglePID.SetMode(AUTOMATIC);
  //Motor Stuff
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);

  // Try to initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //Set up the MPU
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Start timer for MPU and integrating
  long startTime = millis();
  if (Serial1.available() > 0) {
    in_byte = Serial1.read();
    //Serial1.write("Hello");
    if (in_byte == (255)) {
      array_index = 0;
      //Serial.println("We got to changing array index");
    }
    dataIn[array_index] = in_byte;
    //Serial.println("We got to updating array");
    array_index = array_index + 1;
  }
  Xpos = dataIn[2];  //The X position of the joystick
  Ypos = dataIn[3];  //The Y position of the joystick

  button1 = dataIn[1];
  //Serial.println(Xpos);
  //Serial.println(Ypos);
  //Serial.println(button1);
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

  if(degAngle > 180)
  {
    degAngle = (degAngle-360);
  }
  //And the Magnitude is simply a triangle
  double mag = sqrt(sq(trigXpos) + sq(trigYpos));
  /*
Serial.print("The Angle of direction is ");
Serial.println(degAngle);
Serial.print("The magnitude is ");
Serial.println(mag);
Serial.print(trigXpos);
Serial.print(" ");
Serial.println(trigYpos);
*/

  //Getting the actual angle of Roly
  //Insert MPU6050 stuff
  
  
 
  

  
  double errorAngle = degAngle - realAngle;
  errorSum = errorSum + errorAngle;
  //DO some PID stuff
  //Change this back to pid library eventually.
  motorSpeed = (Akp * errorAngle) + (Aki * errorSum) + (Akd * (errorAngle - prevError));

  //anglePID.Compute();

  //Set the motor speeds, currently not trying to drive anywhere, eventually add in some motor speed common to both that is related to magnitude.
  //COnstrained to +-127 so that there can be forward and back
  RWheelSpeed = constrain(baseWheelSpeed - motorSpeed, -127, 127);
  LWheelSpeed = constrain(baseWheelSpeed + motorSpeed, -127, 127);
  //Now change motor speeds to the updated ones
  //drive(RWheelSpeed, LWheelSpeed, mag); Testing


  Serial.print("Right Wheel Speed ");
  Serial.print(RWheelSpeed);
  Serial.print("\t \t Left Wheel Speed ");
  Serial.print(LWheelSpeed);
  Serial.print("\t\t PID Speed: ");
  Serial.print(motorSpeed);
  Serial.print("\t\t Desired Pitch: ");
  Serial.print(degAngle);
  Serial.print("\t\t Actual Pitch: ");
  Serial.print(pitch);
  Serial.print("\r\n");
  delay(10);
  //Update previous error
  prevError = errorAngle;



  //DO the MPU stuff
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  long stopTime = millis();
  long runTime = stopTime - startTime;
  //pitch = 0.96 * g.gyro.roll + 0.04 * g.acceleration.roll;
  pitch = 180*(atan2(-a.acceleration.y, -a.acceleration.z))/3.14;
  Serial.println(pitch);
  
  
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
      Serial.println(rws);
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
