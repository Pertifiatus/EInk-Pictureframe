#include <Stepper.h>                    //load library

#define STEPS 200                       //establish number of steps

Stepper stepper(STEPS, 14, 16, 17, 15);

void setup() {
  // put your setup code here, to run once:

stepper.setSpeed(120);                   //set speed of motor

}

void loop() {
  // put your main code here, to run repeatedly:

stepper.step(300);                      //tell stepper motor to step

}