#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW 0x68   // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH 0x69  // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW
Simple_MPU6050 mpu;
//#define OFFSETS  -5260,    6596,    7866,     -45,       5,      -9  // My Last offsets.
//       You will want to use your own as these are only for my specific MPU6050.
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())  // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!
//Gyro, Accel and Quaternion
void getValues(int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  Serial.printfloatx(F("Yaw"), xyz[0], 9, 4, F(",   "));  //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
  Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(",   "));
  Serial.printfloatx(F("Roll"), xyz[2], 9, 4, F(",   "));
  Serial.printfloatx(F("ax"), accel[0], 5, 0, F(",   "));
  Serial.printfloatx(F("ay"), accel[1], 5, 0, F(",   "));
  Serial.printfloatx(F("az"), accel[2], 5, 0, F(",   "));
  Serial.printfloatx(F("gx"), gyro[0], 5, 0, F(",   "));
  Serial.printfloatx(F("gy"), gyro[1], 5, 0, F(",   "));
  Serial.printfloatx(F("gz"), gyro[2], 5, 0, F("\n"));
  Serial.println();
}
void setup() {
  // put your setup code here, to run once:
  // initialize serial communication
  Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(100);  // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
#ifdef OFFSETS
  Serial.println(F("Using Offsets"));
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
  mpu.load_DMP_Image(OFFSETS);  // Does it all for you

#else
  /* Serial.println(F(" Since no offsets are defined we are going to calibrate this specific MPU6050,\n"
                   " Start by having the MPU6050 placed stationary on a flat surface to get a proper accelerometer calibration\n"
                   " Place the new offsets on the #define OFFSETS... line at top of program for super quick startup\n\n"
                   " \t\t\t[Press Any Key]"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  */
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
  mpu.CalibrateMPU();
  mpu.load_DMP_Image();  // Does it all for you with Calibration
#endif
  mpu.on_FIFO(getValues);  //Print Values is a fcn
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.dmp_read_fifo(false);  // Must be in loop  No Interrupt pin required at the expense of polling the i2c buss
}
