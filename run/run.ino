/* run.ino
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#include "controls.h"
#include "sensing.h"
#include "structs.h"


const uint8_t sensorPinArray [] = {A0, A1, A2};  // from left to right
const int numSensors = sizeof(sensorPinArray) / sizeof(sensorPinArray[0]);
const int sensorRefFrameXR [] = {-4, 0, 4};  // X coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameYR [] = {3, 3, 3};  // Y coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameThetaR [] = {-45, 0, 45};  // angle (degrees) of the sensor reference frames in the robot reference frame R

vec3i refFrames [numSensors];

void setup() {
    initializeSensors(sensorPinArray, &numSensors);
    initializeReferenceFrames(refFrames, &numSensors, vec3iXR, vec3iYR, vec3iThetaR);
}


void loop() {
    int heading = getHeading(sensorPinArray, &numSensors, vec3iXR, vec3iYR, vec3iThetaR);
}