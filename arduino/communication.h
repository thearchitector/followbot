/* communication.h
 *
 * FollowBot POE Project
 * 
 * This header file contains the ROS node Subscriber for receiving heading commands
 * from the RaspberryPi.
 * 
 * @authors: Elias
 */
 
#ifndef communication_h
#define communication_h

#include <ros.h>
#include <std_msgs/UInt16.h>
#include "controls.h"

extern void calculateStrideCoefficients(std_msgs::UInt16&);
extern uint16_t collectDominanteSensorReading();
ros::NodeHandle n;
std_msgs::UInt16 analog_reading;

ros::Subscriber<std_msgs::UInt16> heading("desired_heading", calculateStrideCoefficients);
ros::Publisher ir_sensor("sensor_value", &analog_reading);

void setupSerialNode() {
  n.initNode();
  n.subscribe(heading);
  n.advertise(ir_sensor);
}

void communicate() {
  analog_reading.data = collectDominanteSensorReading();
  ir_sensor.publish(&analog_reading);
  n.spinOnce();
}

#endif //communication_h
