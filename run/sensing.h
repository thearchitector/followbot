/* sensing.h
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan and Kristin
 */

#ifndef sensing_h
#define sensing_h

#include "conversions.h"

// TODO: Verify
typedef struct sensorRefFrame {
    union {
        int vals [3];
        struct {
            int xR; 
            int yR; 
            int thetaR;
        };
    };
} sensorRefFrame;


void initializeReferenceFrames(sensorRefFrame *refFrames, const int *numSensors, const int sensorRefFrameXR[], 
               const int sensorRefFrameYR[], const int sensorRefFrameThetaR[]) {
    /*
     * Initializes all sensor reference frames
     */
    for (int s=0; s < *numSensors; s++) {
        refFrames[s].xR = sensorRefFrameXR[s];
        refFrames[s].yR = sensorRefFrameYR[s];
        refFrames[s].thetaR = sensorRefFrameThetaR[s];
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


int getHeading(const uint8_t sensorPinArray[], const int *numSensors, const int sensorRefFrameXR[], 
               const int sensorRefFrameYR[], const int sensorRefFrameThetaR[]) {
    /*
     * Computes the desired heading of the robot relative to its reference frame. This heading is in degrees
     */

    // calculate repulsive forces
    int repulsiveForces [*numSensors];
    for (int s=0; s < *numSensors; s++) {
        repulsiveForces[s] = analogInToRepulsive(analogRead(sensorPinArray[s]));
    }
    

}


#endif