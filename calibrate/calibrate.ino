/*
 * Lab 3 of POE
 *
 * Calibration code for line following robot. Instructions for use are printed to serial
 * when ran.
 * 
 * @authors: Dunan Mazza and Megan Ku
 *  (note: This code was adapted from Duncan's 2nd POE lab, but was almost exclusively 
 *   originally authored by Duncan)
 */

const uint8_t sensorInPin = A3;  // Analog in pin for the sensor

const char buttonInPin = 2;  // Digital in pin for the button (must be interrupt-capable)
volatile bool flag = false;  // flag used for controlling procession through data collection 
// loop (switched by user pressing the button)

void makeFlagTrue() {
  /*
   * Callback for pin interrupt; makes flag true if it is false
   */
  if (!flag) flag = true;
}

// Calibration distance values for relating distance (in mm) to measured analog in values
const int mm_calib[] = {0, 2, 4, 6, 8, 10, 12, 14, 16};

const int num_datapoints_at_each = 10; // number of data points to take at each distance

unsigned int meanOfArray(unsigned int array_for_means[]) {
  /*
   * Calculates the mean of the array passed in, where this array has num_datapoints values
   */
  unsigned int mean = 0;
  for (int j = 0; j < num_datapoints_at_each; j++) {
    mean += array_for_means[j];
  }
  return mean / num_datapoints_at_each;
}

void setup() {
  Serial.begin(9600);

  // initialize pins
  pinMode(sensorInPin, INPUT);
  pinMode(buttonInPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(buttonInPin), ::makeFlagTrue, RISING);

  const int mm_calib_len = sizeof(mm_calib) / sizeof(mm_calib[0]);  // 
  unsigned int analog_in_calib[mm_calib_len]; // initialize measurement array
  unsigned int array_for_means[num_datapoints_at_each]; // array used to calculate the 
  // mean value of data measurements over a short period of time

  Serial.print("You have defined ");
  Serial.print(mm_calib_len);
  Serial.println(" distances to measure at");
  Serial.println("");

  for (int i; i < mm_calib_len; i++) {
    Serial.print("Calibrating at distance ");
    Serial.println(mm_calib[i]);
    Serial.println("Press the button when you are ready...");

    while (!flag); // wait for flag to be turned true by button button press

    // take num_datapoints_at_each data points
    for (int j = 0; j < num_datapoints_at_each; j++) {
      array_for_means[j] = analogRead(sensorInPin);  // read and store data value
      Serial.print("    analog-in value = ");
      Serial.println(array_for_means[j]);  // display analog-in value
      delay(100);
    }
    analog_in_calib[i] = meanOfArray(array_for_means);  // calculate and store mean 
    // data value collected
    Serial.print("Mean analog-in value calculated to be: ");
    Serial.println(analog_in_calib[i]);  // display mean data value collected
    Serial.println("");
    flag = false; // switch back flag to false for next loop
  }

  // Print out results to the terminal
  Serial.println("");
  Serial.println("The sensor is now calibrated. Copy the data from the following array into scan.ino");
  Serial.println("");
  Serial.print("[");
  // print out all of the mean data values calculated in a MATLAB matrix format (can be copied and pasted
  // into MATLAB)
  for (int i; i <= mm_calib_len - 1; i++) {
    Serial.print(analog_in_calib[i]);
    if (i < mm_calib_len - 1) Serial.print(", ");
  }
  Serial.println("]");
}

void loop() {
  // n/a - press reset button on Arduino to take another measurement
}