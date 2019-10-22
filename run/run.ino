/* run.ino
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#include "controls.h"
#include "sensing.h"


const uint8_t sensorPinArray [] = {A0, A1, A2};
const int numSensors = sizeof(sensorPinArray) / sizeof(sensorPinArray[0]);

void setup() {
    initializeSensors(&sensorPinArray);
}


void loop() {

}