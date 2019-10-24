/* conversions.h
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#ifndef conversions_h
#define conversions_h

#include "structs.h"

int analogInToSensorDistance(int analogIn) {
    int sensorDistance = 0;
    return sensorDistance;
}

int sensorDistanceToRobotFrameDistance(int sensorDistance) {
    int robotFrameDistance = 0;
    return robotFrameDistance;
}

int robotFrameDistanceToRepulsive(int robotFrameDistance) {
    int repulsive = 0;
    return repulsive;
}

int analogInToRepulsive(int analogIn) {
    return robotFrameDistanceToRepulsive(sensorDistanceToRobotFrameDistance(analogInToSensorDistance(analogIn)));
}

int getVec3iAngle(vec3i vec) {
    return;
}

#endif