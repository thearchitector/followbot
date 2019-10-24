/* 
 * This source file contains all the logic responsible for moving the robot given a
 * specific heading and magnitude.
 * 
 * @authors: Elias
 */

#include "Arduino.h"
#include "controls.h"

const uint8_t RANGE_OF_MOTION = 90;
const uint8_t NUM_SERVOS = 4;
const uint8_t SERVO_PINS[] = {0, 1, 2, 3};
const Servo SERVOS[4];

double leftStrideCoefficient;
double rightStrideCoefficient;
int currentStrideAngle = 0;
bool stepping = true;

void attachServos() {
  for(int i = 0; i < NUM_SERVOS; i++) {
    SERVOS[i].attach(SERVO_PINS[i]);
    SERVOS[i].write(0);
  }
}

void calculateStrideCoefficients(uint16_t heading, uint8_t magnitude) {
  if(heading > 180) {
    heading -= 180;
    magnitude *= -1;
  }

  leftStrideCoefficient = heading / 180.0;
  rightStrideCoefficient = 1 - leftStride;
}

void moveAtHeading() {
  SERVOS[0].write(floor(leftStrideCoefficient * currentStrideAngle));
  SERVOS[1].write(floor(rightStrideCoefficient * (RANGE_OF_MOTION - currentStrideAngle)));
  SERVOS[2].write(i);
  SERVOS[3].write(RANGE_OF_MOTION - currentStrideAngle);
  
  currentStrideAngle = currentStrideAngle + (stepping ? 1 : -1);
  if(currentStrideAngle == RANGE_OF_MOTION || currentStrideAngle == 0) stepping = !stepping;
  delay(15);
}
