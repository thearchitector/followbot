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

const uint8_t STRIDE_RANGE = 180;
const uint8_t STEP_RANGE = 45;
const uint8_t NUM_SERVOS = 4;
const uint8_t SERVO_PINS[] = {11, 10, 9, 8};
const Servo SERVOS[4];

double leftStrideCoefficient;
double rightStrideCoefficient;
int currentStrideAngle = 45;
int currentStepAngle = 0;
uint8_t dir = 1;
bool strideDirection = true;
bool stepDirection = true;

void attachServos(uint16_t heading) {
    currentStrideAngle = heading;

    for(int i = 0; i < NUM_SERVOS; i++) {
        SERVOS[i].attach(SERVO_PINS[i]);
        SERVOS[i].write(heading);
    }
}

void calculateStrideCoefficients(uint16_t heading) {
    dir = 1;

    if(heading > 180) {
        heading -= 180;
        dir = -1;
    }

    rightStrideCoefficient = heading / 180.0;
    leftStrideCoefficient = 1 - rightStrideCoefficient;
}

void moveAtHeading() {
    SERVOS[0].write(floor(leftStrideCoefficient * currentStrideAngle));
    SERVOS[1].write(floor(rightStrideCoefficient * currentStrideAngle));
    SERVOS[2].write(STEP_RANGE - currentStepAngle);
    SERVOS[3].write(currentStepAngle);
  
    currentStrideAngle = currentStrideAngle + (strideDirection ? dir : -dir);
    currentStepAngle = currentStepAngle + (strideDirection ? dir : -dir);

    if(currentStrideAngle == STRIDE_RANGE || currentStrideAngle == 0) strideDirection = !strideDirection;
    if(currentStepAngle == STEP_RANGE || currentStepAngle == 0) stepDirection = !stepDirection;

    delay(15);
}

#endif