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

void attachServos();
void calculateStrideCoefficients(uint16_t heading);
void moveAtHeading();

#endif
