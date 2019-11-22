/* sensing.h
 *
 * FollowBot POE Project
 * 
 * @authors: Duncan, Elias
 */

#ifndef sensing_h
#define sensing_h

const uint8_t NUM_SENSORS = 4;
const uint8_t SENSOR_PINS[] = {A2, A1, A0};
const uint16_t MIN_ANALOG_CUTOFF = 100;

void initializeSensors() {
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        pinMode(SENSOR_PINS[i], INPUT);
    }
}

uint16_t collectDominanteSensorReading() {
    uint16_t largestSensorValue = MIN_ANALOG_CUTOFF;

    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        uint16_t sensorValue = analogRead(SENSOR_PINS[i]);
        if (sensorValue > largestSensorValue) largestSensorValue = sensorValue;
    }

    return largestSensorValue;
}

#endif
