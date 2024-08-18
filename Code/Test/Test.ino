void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  pinMode(8, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial1.available())
  {
    String testString = Serial1.readString();
    digitalWrite(8, HIGH);
    delay(5000);
    digitalWrite(8, LOW);
    Serial.println("We received some stuff");
    Serial.println(testString);
  }


}
