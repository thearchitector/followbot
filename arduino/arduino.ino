/* arduino_run.ino

   FollowBot POE Project

   @authors: Duncan, Elias
*/

#include "controls.h"
#include "sensing.h"

void setup() {
  Serial.begin(9600);
  attachServos(90, 90);
  initializeSensors();
}

void loop() {
  moveAtHeading();
  communicate();
}
