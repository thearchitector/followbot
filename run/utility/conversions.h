/* conversions.h
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#ifndef conversions_h
#define conversions_h

#include "structs.h"
#include <math.h>

int rad2Deg(double rad) {
    return (int)(180. * rad / PI);
}

double deg2Rad(int deg) {
    return PI * (double)deg / 180.;
}

// TODO
int analogInToSensorDistance(int analogIn) {
    int sensorDistance = 0;
    return sensorDistance;
}

int distanceToRepulsive(int distance) {
    int repulsive;
    if (distance < 30) {
        repulsive = 100;
    } else if (distance >= 30 && distance <= 60) {
        repulsive = -distance + 60;
    } else {
        repulsive = 0;
    }
    return repulsive;
}

int analogInToRepulsive(int analogIn) {
    return distanceToRepulsive(analogInToSensorDistance(analogIn));
}



#endif