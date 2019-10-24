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

extern const int minAnalogInCutoff;

int rad2Deg(double rad) {
    return (int)(180. * rad / PI);
}

double deg2Rad(int deg) {
    return PI * (double)deg / 180.;
}

int analogInToRepulsive(int analogIn) {
    return analogIn < minAnalogInCutoff ? 0 : analogIn;    
}


#endif