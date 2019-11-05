/* arduino_run.ino

   FollowBot POE Project

   @authors: Duncan and Kristin
*/

#include "controls/controls.h"
#include "sensing/sensing.h"
#include "utility/structs.h"


const uint8_t sensorPinArray [3] = {A2, A1, A0};  // from left to right
const int numSensors = sizeof(sensorPinArray) / sizeof(sensorPinArray[0]);

// The following arrays store parameters for the sensor positions; arrays contain information in order from the left sensor to the
// right sensor.
const int sensorRefFrameXR [3] = { -4, 0, 4}; // X coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameYR [3] = {3, 3, 3};  // Y coordinates (cm) of the sensor referance frames in the robot reference frame R
const int sensorRefFrameThetaR [3] = {315, 270, 225};  // angle (degrees) of the sensor reference frames in the robot reference frame R,
// pointing inwards towards the robot (in the opposite direction of the sensor)
vec3i refFrames [numSensors];

// rate at which to poll the sensors and calculate a new heading
const int sensorCounterThreshold = 10;
int sensorCounter = sensorCounterThreshold;

int attractiveForce = 300;
int attractiveHeading = 90;
const int minAnalogInCutoff = 100;
const int repulsiveFloor = 0;
const int maxAnalogInCutoff = 500;
const int repulsiveCeiling = 10000;

uint16_t heading = 90;  // initialize heading

void setup() {
  Serial.begin(9600);
  attachServos(90, 90);
  calculateStrideCoefficients(&heading);  // set motors to an initial position
//  initializeSensors(sensorPinArray, &numSensors);
//  initializeReferenceFrames(refFrames, &numSensors, sensorRefFrameXR, sensorRefFrameYR, sensorRefFrameThetaR);
}

void loop() {
  // if (sensorCounter == sensorCounterThreshold) {
  //     heading = getHeading(sensorPinArray, &numSensors, refFrames);
  //     Serial.print("heading:");
  //     Serial.println(heading);
  //     sensorCounter = 0;
  // }
  // sensorCounter++;
  moveAtHeading();
}
