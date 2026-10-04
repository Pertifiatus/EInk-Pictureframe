void setup() {
  // put your setup code here, to run once:
pinMode(47,OUTPUT);
pinMode(38,OUTPUT);
pinMode(18,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(47,1);
digitalWrite(38,0);
digitalWrite(18,0);
delay(1000);
digitalWrite(47,0);
digitalWrite(38,1);
digitalWrite(18,0);
delay(1000);
digitalWrite(47,0);
digitalWrite(38,0);
digitalWrite(18,1);
delay(1000);
digitalWrite(47,0);
digitalWrite(38,0);
digitalWrite(18,0);
delay(1000);
}
