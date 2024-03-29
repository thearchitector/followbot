/* arduino.ino
 * FollowBot POE Project
 * @authors: Duncan, Elias
 *
 * Main routine for the Aduino Uno motor controller funcitonality
 *
*/

#include <ros.h>
#include <std_msgs/UInt16.h>
#include "controls.h"

ros::NodeHandle n;
ros::Subscriber<std_msgs::UInt16> heading{"desired_heading", calculateStrideCoefficients};

void setup() {
  Serial.println(9600);
  attachServos(90, 90);
  n.initNode();
  n.subscribe(heading);
}

void loop() {
  moveAtHeading();
  n.spinOnce();
}
