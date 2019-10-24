/* coms.h
 *
 * FollowBot POE Project
 * 
 * This header file contains ... TODO
 * 
 * @authors: Duncan Mazza
 */


#ifndef coms_h
#define coms_h



void listen() {
    /*
     * Listens to serial communications and modifies relevant values as needed. Commands
     * are read as a series of alphanumeric characters that starts with a key value that is
     * a letter and is immediately followed by a string of digits that, when combined, 
     * make an integer value. Any character that is received that does not fit this pattern 
     * is ignored. Negative integers could be supported as an input by checking for a "-" 
     * preceding the number, but this feature is not supported as none of the variables that
     * are currently being tuned should be >0. 
     * 
     * Accepted keys:
     *  - "a" (alias for insideWheelSpeed variable)
     *  - "b" (alias for onLine variable)
     *  - "c" (alias for defaultSpeed variable)
     * 
     * Example commands:  (note that "\n" is implicit in hitting enter when sending data)
     *  - "a5\n" changes insideWheelSpeed to 5
     *  - "b700\n" changes onLine to 700
     *  - "c30\n" changes defaultSpeed to 30
     *  - "c-30\n" changes defaultSpeed to 30 (negative sign is ignored)
     *  - "d32\n" imparts no change because 'd' is not a supported key
     */

    String inString = "";  // stores the digits in the incoming communication 
    char key;  // key that selects the variable to be altered
    int value;  // value to set the selected variable to

    if (Serial.available() > 0) {  // if there is data in the buffer
        delay(10);  // this delay is needed or else the arduino will only recognize one 
        // character in its buffer at a time
        while (Serial.available() > 0) {
            char inChar = (char)Serial.read();  // typecast the returned integer to a character
            if (inChar == keyInsideWheelSpeed || inChar == keyOnLine || inChar == keyDefaultSpeed) {
                // character is one of the key values 
                key = inChar;
            } else if (isDigit(inChar)) {
                // character is a number
                inString += inChar;
            }  // else {any unrecognized characters are ignored}
        }

        value = inString.toInt();  // convert the string of digits stored as characters to an integer

        // change the selected variable to the input value, and print a confirmation 
        if (key == keyInsideWheelSpeed) {
            Serial.print("Changed insideWheelSpeed value to ");
            insideWheelSpeed = value;
        } else if (key == keyOnLine) {
            Serial.print("Changed onLine value to ");
            onLine = value;
        } else if (key == keyDefaultSpeed) {
            Serial.print("Changed defaultSpeed value to ");
            defaultSpeed = value;
        } else {
            Serial.print("Key received = ");
            Serial.print(key);
            Serial.print("; nothing changed; value received = ");
        }
        Serial.println(value);
    }
}


#endif