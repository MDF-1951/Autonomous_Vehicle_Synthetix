#include <Servo.h>
#include <AFMotor.h>
//velocity at 150 of motor = 41.9cm/s
Servo myservo;
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);


int pos = 55;    // variable to store the servo position
int a=1; //flag for steering
int i=0;
int end=0;
int end_true=0;
void setup() {
  myservo.attach(10); 
  Serial.begin(9600);
}

void loop() {
  if(end==0){
  int t1=millis();
  motor1.run(FORWARD);
  motor1.setSpeed(150);
  motor2.run(FORWARD);
  motor2.setSpeed(150); 
  if(millis()<=1432){
  myservo.write(55); //pos=60
  }
  //initiating steer
  if(millis()>1432 && i==0){
    int t2=millis();
    for(pos=55; pos>=0; pos-=5){
      myservo.write(pos);
      delay(160);
    }
    i=1;
    int t3=millis();
  } 
  //returning to normal steer
  if(millis()>1432 && i==1){
    int t4=millis();
    for(pos=0; pos<=55; pos+=5){
      myservo.write(pos);
      delay(100);
    }
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    int t5=millis();
    end=1;
  } 
}
}
