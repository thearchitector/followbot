#ifndef PLAN_HEADING_HPP
#define PLAN_HEADING_HPP

#include <ros/ros.h>
#include <astar.hpp>
#include <followbot/Point2.h>
#include <followbot/Buffer.h>
#include <std_msgs/UInt16.h>
#include <iostream>
#include "std_msgs/String.h"

void human_pose_callback(const followbot::Buffer &buffer_msg);
void buffer_callback(const followbot::Buffer &buffer_msg);

#endif // PLAN_HEADING_HPP