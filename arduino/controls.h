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

#include <Servo.h>

const uint8_t NUM_SERVOS = 4;
const uint8_t SERVO_PINS[] = {11, 10, 9, 8};
Servo SERVOS[NUM_SERVOS];
const double MIN_STRIDE = 45;
const double MAX_STRIDE = 135;
const double STRIDE_RANGE = MAX_STRIDE - MIN_STRIDE;
const uint8_t MIN_STEP = 80;
const uint8_t MAX_STEP = 100;

const uint8_t INDICATOR_PIN = 7;
const uint16_t STOPSIG = 65535;

double currentStrideAngle;
double currentStepAngle;

double leftStrideCoefficient = 0.5;
double rightStrideCoefficient = 0.5;

bool strideDirection = true;
bool stepDirection = true;

bool personDetected = false;

void attachServos(double angle0, double angle1) {
    double pos[] = {angle0, angle0, angle1, angle1};
    currentStrideAngle = angle0;
    currentStepAngle = angle1;

    for(int i = 0; i < NUM_SERVOS; ++i) {
        SERVOS[i].attach(SERVO_PINS[i]);
        SERVOS[i].write(pos[i]);
    }

    pinMode(INDICATOR_PIN, OUTPUT);
}

bool isReverse = false;
double dir = 1;

void calculateStrideCoefficients(const std_msgs::UInt16& msg) {
    personDetected = msg.data != STOPSIG;

    digitalWrite(INDICATOR_PIN, personDetected ? HIGH : LOW);

    if(personDetected) {
        double heading = (double)msg.data;

        if(heading > 180) {
            heading -= 180;
    
            if(!isReverse) {
                dir = -1;
                isReverse = true;
            }
        }
        else if(isReverse) {
            dir = 1;
            isReverse = false;
        }
    
        rightStrideCoefficient = heading / 180.0;
        leftStrideCoefficient = 1 - rightStrideCoefficient; 
    }
}

void moveAtHeading() {
    if(personDetected) {
        bool flag = false;
        double lMin = 90 - (STRIDE_RANGE / (2 / leftStrideCoefficient));
        double lMax = 90 + (STRIDE_RANGE / (2 / leftStrideCoefficient));
        double rMin = 90 - (STRIDE_RANGE / (2 / rightStrideCoefficient));
        double rMax = 90 + (STRIDE_RANGE / (2 / rightStrideCoefficient));
    
        SERVOS[0].write(max(min(currentStrideAngle, lMax), lMin));
        SERVOS[1].write(max(min(currentStrideAngle, rMax), rMin));
    
        currentStrideAngle = currentStrideAngle + (strideDirection ? 1 : -1);
    
        if(currentStrideAngle == MAX_STRIDE || currentStrideAngle == MIN_STRIDE) {
            strideDirection = !strideDirection;
    
            while(true) {
                SERVOS[2].write(currentStepAngle);
                SERVOS[3].write(currentStepAngle);
    
                currentStepAngle = max(min(currentStepAngle + (stepDirection ? dir : -dir), MAX_STEP), MIN_STEP);
    
                if(currentStepAngle == MAX_STEP || currentStepAngle == MIN_STEP) {
                    stepDirection = !stepDirection;
                    break;
                }
    
                delay(3);
            }
        }
    }
    else {
        SERVOS[0].write(90);
        SERVOS[1].write(90);
        SERVOS[3].write(90);
        SERVOS[4].write(90);
    }

    delay(3);
}

#endif
