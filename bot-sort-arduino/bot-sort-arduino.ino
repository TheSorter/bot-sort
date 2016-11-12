#include <Servo.h>

Servo twist, claw, height, extend;
const int twist_max = 180;
const int twist_min = 0;
const int twist_default = 90;

const int claw_max = 175;
const int claw_min = 35;
const int claw_default = 150;

const int height_max = 140;
const int height_min = 30;
const int height_default = 100;

const int extend_max = 145;
const int extend_min = 100;
const int extend_default = 100;

void hit_bottle_left(){
  digitalWrite(8, HIGH);
  twist.write(45);
  delay(500);
  extend.write(140);
  delay(500);
  twist.write(90);
  delay(500);
  extend.write(90);
  delay(500);
  digitalWrite(8, LOW);
}

void hit_bottle_right(){
  digitalWrite(8, HIGH);
  twist.write(135);
  delay(500);
  extend.write(130);
  delay(500);
  twist.write(90);
  delay(500);
  extend.write(90);
  delay(500);
  digitalWrite(8, LOW);
}

void hit_bottle_forward(){
  digitalWrite(8, HIGH);
  height.write(80);
  delay(500);
  extend.write(130);
  delay(500);
  extend.write(90);
  delay(500);
  height.write(height_default);
  digitalWrite(8, LOW);
}

void setup() {
  twist.attach(3);
  claw.attach(9);
  height.attach(5);
  extend.attach(6);
  twist.write(twist_default);
  delay(100);
  claw.write(claw_default);
  delay(100);
  height.write(height_default);
  delay(100);
  extend.write(extend_default);
  delay(100);
  Serial.begin(9600);
  Serial.println("serial started");
}

void loop() {
  int result = -1;
  while(Serial.available()){
    result = Serial.read();
    result -= 48;
    if(result == 10){
      result = -1;
      continue;
    }
  }
  if(result > 0){
    Serial.println("got number, it was: ");
    Serial.println(result);
    switch(result){
      case 1:
        hit_bottle_left();
        break;
      case 2:
        hit_bottle_right();
        break;
      case 3:
        hit_bottle_forward();
        break;
    }
  }

}
