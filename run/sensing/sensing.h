/* sensing.h
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#ifndef sensing_h
#define sensing_h

#include "../utility/conversions.h"
#include "../utility/structs.h"

// Stand-in for sensing human - sets the attractive force and heading to a constant:
extern int attractiveForce;
extern int attractiveHeading;

void initializeReferenceFrames(vec3i *refFrames, const int *numSensors, const int vec3iXR[], 
               const int vec3iYR[], const int vec3iThetaR[]) {
    /*
     * Initializes all sensor reference frames
     */
    for (int s=0; s < *numSensors; s++) {
        refFrames[s].xR = vec3iXR[s];
        refFrames[s].yR = vec3iYR[s];
        refFrames[s].thetaR = vec3iThetaR[s];
    }
}

void initializeSensors(const uint8_t sensorPinArray[], const int *numSensors) {
    /*
     * Initializes all analog in sensor pins
     */
    for (int s=0; s < *numSensors; s++) {
        pinMode(sensorPinArray[s], INPUT);
    }
}


int getVectorSumHeading(int *repulsiveForce, int *repulsiveHeading, int *attractiveForce, int *attractiveHeading) {
    double repulsiveXY [2] = {*repulsiveForce * cos(deg2Rad(*repulsiveHeading)),
                              *repulsiveForce * sin(deg2Rad(*repulsiveHeading))};
    double attractiveXY [2] = {*attractiveForce * cos(deg2Rad(*attractiveHeading)), 
                               *attractiveForce * sin(deg2Rad(*attractiveHeading))};
    return rad2Deg(atan((repulsiveXY[1] + attractiveXY[1]) / (repulsiveXY[0] + attractiveXY[0])));
}


int getHeading(const uint8_t sensorPinArray[], const int *numSensors, vec3i refFrames[]) {
    /*
     * Computes the desired heading of the robot relative to its reference frame. This heading is in degrees
     */

    // calculate repulsive force and heading
    int largestSensorRead = 0;
    int indexOfLargestSensorRead;
    
    for (int s=0; s < *numSensors; s++) {
        int sensorRead = analogRead(sensorPinArray[s]);
        if (sensorRead > largestSensorRead) {
            largestSensorRead = sensorRead;
            indexOfLargestSensorRead = s;
        }
    }
    int repulsiveForce = analogInToRepulsive(largestSensorRead);
    int repulsiveHeading = refFrames[indexOfLargestSensorRead].thetaR;

    return getVectorSumHeading(&repulsiveForce, &repulsiveHeading, &attractiveForce, &attractiveHeading);
}


#endif