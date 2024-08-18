#define redPin 9
#define greenPin 10
#define bluePin 11
#define PWMA 5
#define AIN2 21
#define AIN1 20
#define BIN1 19
#define BIN2 18
#define PWMB 6
#define STDBY 10


void setup() {

  Serial.begin(9600); // Default communication rate of the Bluetooth module
   Serial1.begin(9600);
   pinMode(AIN1, OUTPUT);
   pinMode(AIN2, OUTPUT);
   pinMode(BIN1, OUTPUT);
   pinMode(BIN2, OUTPUT);
   pinMode(PWMA, OUTPUT);
   pinMode(PWMB, OUTPUT);
   pinMode(STDBY, OUTPUT);
   digitalWrite(STDBY, HIGH);
}


void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    // Check the received command and update the Directions accordingly
    if (command.length() == 9) {
      // Extract the values from the command
      int redValue = command.substring(0, 3).toInt();
      int greenValue = command.substring(3, 6).toInt();
      int blueValue = command.substring(6).toInt();

      //Determine Overall speed
      float LWheelMultiplier = float(redValue)/255;
      float RWheelMultiplier = float(blueValue)/255;
      float speed = 255-float(greenValue);
      float RWheelSpeed;
      float LWheelSpeed;
      if(speed>30)
      {
      RWheelSpeed = (speed*RWheelMultiplier);
      LWheelSpeed = (speed*LWheelMultiplier);
      }
      else if((speed > 1) && (speed < 30))
      {
        RWheelSpeed = 0;
        LWheelSpeed = 0;
      }

      else
      {
        RWheelSpeed = -(255*(1 - LWheelMultiplier));
        LWheelSpeed = -(255*(1- RWheelMultiplier)); //Changed (FLipped) these to reflect mirrored, hope it works 
      }
      
      Serial.println(LWheelMultiplier);
      Serial.print(LWheelSpeed);
      Serial.println(" Left Motor");
      Serial.println(RWheelMultiplier);
      Serial.print(RWheelSpeed);
      Serial.println(" Right Motor");
      Serial.print(speed);
      Serial.println( "Speed");
      drive(RWheelSpeed, LWheelSpeed, speed);

      delay(50);

      
    }
  }
}

void drive(float rightWheelSpeed, float leftWheelSpeed, float speed) {
  if(speed > 30)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMA, int(rightWheelSpeed*0.5));
    analogWrite(PWMB, int(leftWheelSpeed*0.5));
  }
  else if((rightWheelSpeed == 0)&& (leftWheelSpeed == 0))
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  
  else
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, int(-rightWheelSpeed*0.5));
    analogWrite(PWMB, int(-leftWheelSpeed*0.5));
  }
 }

