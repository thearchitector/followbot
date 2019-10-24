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

// The following arrays store parameters for the sensor positions; arrays contain information in order from the left sensor to the 
// right sensor.
const int sensorRefFrameXR [3] = {-4, 0, 4};  // X coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameYR [3] = {3, 3, 3};  // Y coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameThetaR [3] = {-45, -90, -135};  // angle (degrees) of the sensor reference frames in the robot reference frame R,
// pointing inwards towards the robot (in the opposite direction of the sensor)

int attractiveForce = 20;
int attractiveHeading = 90;
const int minAnalogInCutoff = 100;

vec3i refFrames [numSensors];

void setup() {
    Serial.begin(9600);

    initializeSensors(sensorPinArray, &numSensors);
    initializeReferenceFrames(refFrames, &numSensors, sensorRefFrameXR, sensorRefFrameYR, sensorRefFrameThetaR);
}

void loop() {
    int heading = getHeading(sensorPinArray, &numSensors, refFrames);
    Serial.println(heading);
}