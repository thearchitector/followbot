/* controls.h
 *
 * FollowBot POE Project
 * 
 * This header file contains all the constants, buildtime, and runtime variables necessary
 * to compile and execute the robot controls.
 * 
 * @authors: Elias
 */

#ifndef controls_h
#define controls_h

#include <stdint.h>
#include <Arduino.h>
#include <Servo.h>

// void attachServos();
// void calculateStrideCoefficients(uint16_t heading);
// void moveAtHeading();

const uint8_t RANGE_OF_MOTION = 90;
const uint8_t NUM_SERVOS = 4;
const uint8_t SERVO_PINS[] = {11, 10, 9, 8};
const Servo SERVOS[4];

double coefficients [2];
int currentStrideAngle = 45;
uint8_t dir = 1;
bool stepping = true;

void attachServos() {
  for(int i = 0; i < NUM_SERVOS; i++) {
    SERVOS[i].attach(SERVO_PINS[i]);
    SERVOS[i].write(45);
  }
}

double * calculateStrideCoefficients(int *heading) {
  dir = 1;

  // TODO: make more efficient
  if(*heading > 180) {
    *heading -= 180;
    dir = -1;
  }

  coefficients[1] = (double)*heading / 180.0;
  coefficients[0] = (1 - coefficients[1]);
  return coefficients;
}

void moveAtHeading(int *heading) {
  calculateStrideCoefficients(heading);

  SERVOS[0].write(floor(coefficients[0] * currentStrideAngle));
  SERVOS[1].write(floor(coefficients[1] * (RANGE_OF_MOTION - currentStrideAngle)));
  SERVOS[2].write(RANGE_OF_MOTION - currentStrideAngle);
  SERVOS[3].write(currentStrideAngle);
  
  currentStrideAngle = currentStrideAngle + (stepping ? dir : -dir);
  if(currentStrideAngle == RANGE_OF_MOTION || currentStrideAngle == 0) stepping = !stepping;
  delay(15);
}

#endif
