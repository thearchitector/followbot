/* run.ino
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#include "controls/controls.h"
#include "sensing/sensing.h"
#include "utility/structs.h"


const uint8_t sensorPinArray [3] = {A0, A1, A2};  // from left to right
const int numSensors = sizeof(sensorPinArray) / sizeof(sensorPinArray[0]);
const int sensorRefFrameXR [3] = {-4, 0, 4};  // X coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameYR [3] = {3, 3, 3};  // Y coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameThetaR [3] = {-45, 0, 45};  // angle (degrees) of the sensor reference frames in the robot reference frame R

int attractiveForce = 20;
int attractiveHeading = 90;

vec3i refFrames [numSensors];

void setup() {
    Serial.begin(9600);
    attachServos();
    calculateStrideCoefficients(45);
//    initializeSensors(sensorPinArray, &numSensors);
//    initializeReferenceFrames(refFrames, &numSensors, sensorRefFrameXR, sensorRefFrameYR, sensorRefFrameThetaR);
}

void loop() {
//    int heading = getHeading(sensorPinArray, &numSensors, refFrames);
//    Serial.println(heading);
}
