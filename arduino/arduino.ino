/* arduino_run.ino

   FollowBot POE Project

   @authors: Duncan, Elias
*/

#include <ros.h>
#include <std_msgs/UInt16.h>
#include "controls.h"

ros::NodeHandle n;
ros::Subscriber<std_msgs::UInt16> heading("desired_heading", calculateStrideCoefficients);

void setup() {
  attachServos(90, 90);
//  n.initNode();
//  n.subscribe(heading);
}

void loop() {
//  moveAtHeading();
//  n.spinOnce();
}
