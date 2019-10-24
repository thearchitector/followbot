/* sensing.h
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#ifndef sensing_h
#define sensing_h

#include "conversions.h"
#include "structs.h"

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


int getHeading(const uint8_t sensorPinArray[], const int *numSensors, const int vec3iXR[], 
               const int vec3iYR[], const int vec3iThetaR[]) {
    /*
     * Computes the desired heading of the robot relative to its reference frame. This heading is in degrees
     */

    // calculate repulsive forces
    int smallestAnalogIn = 0;
    
    for (int s=0; s < *numSensors; s++) {
        
    }
    int repulsiveForce = analogInToRepulsive(smallestAnalogIn));
    
    

}


#endif