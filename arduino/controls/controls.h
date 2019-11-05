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

const uint8_t MIN_STRIDE = 45;
const uint8_t MAX_STRIDE = 135;
const uint8_t STRIDE_ORIGIN = (MAX_STRIDE - MIN_STRIDE) / 2;
const uint8_t MIN_STEP = 80;
const uint8_t MAX_STEP = 110;
const uint8_t NUM_SERVOS = 4;
const uint8_t SERVO_PINS[] = {11, 10, 9, 8};
const Servo SERVOS[NUM_SERVOS];

double leftStrideCoefficient;
double rightStrideCoefficient;
uint8_t currentStrideAngle = 90;
uint8_t currentStepAngle = 90;
uint8_t dir = 1;
bool strideDirection = true;
bool stepDirection = true;

void attachServos(uint8_t angle0, uint8_t angle1) {
    uint8_t pos[] = {angle0, angle0, angle1, angle1};
    currentStrideAngle = angle0;
    currentStepAngle = angle1;

    for(int i = 0; i < NUM_SERVOS; i++) {
        SERVOS[i].attach(SERVO_PINS[i]);
        SERVOS[i].write(pos[i]);
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
    SERVOS[0].write((currentStrideAngle * leftStrideCoefficient) + STRIDE_ORIGIN);
    SERVOS[1].write((currentStrideAngle * rightStrideCoefficient) + STRIDE_ORIGIN);

    currentStrideAngle = currentStrideAngle + (strideDirection ? dir : -dir);

    if(currentStrideAngle == MAX_STRIDE || currentStrideAngle == MIN_STRIDE) {
        strideDirection = !strideDirection;

        while(true) {
            SERVOS[2].write(currentStepAngle);
            SERVOS[3].write(currentStepAngle);

            currentStepAngle = currentStepAngle + (stepDirection ? dir : -dir);

            if(currentStepAngle == MAX_STEP || currentStepAngle == MIN_STEP) {
                stepDirection = !stepDirection;
                break;
            }

            delay(3);
        }
    }

    delay(3);
}

#endif
