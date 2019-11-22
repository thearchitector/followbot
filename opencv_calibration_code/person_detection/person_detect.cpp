/* person_detect.cpp
 * @author: Duncan Mazza
 *
 * This code has been adapted (albeit with significant modifications) from code written at BigVision LLC. Their
 * copyright notice is reproduced below:
 *      This code is written at BigVision LLC. It is based on the OpenCV project. It is subject to the license terms in
 *      the LICENSE file found in this distribution and at http://opencv.org/license.html
 * The contents of that license are reproduced below:

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
 license, do not download, install, copy or use the software.

License Agreement
For Open Source Computer Vision Library
(3-clause BSD License)

Copyright (C) 2000-2019, Intel Corporation, all rights reserved.
Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
Copyright (C) 2009-2016, NVIDIA Corporation, all rights reserved.
Copyright (C) 2010-2013, Advanced Micro Devices, Inc., all rights reserved.
Copyright (C) 2015-2016, OpenCV Foundation, all rights reserved.
Copyright (C) 2015-2016, Itseez Inc., all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 following conditions are met:
  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following
    disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
    the following disclaimer in the documentation and/or other materials provided with the distribution.
  - Neither the names of the copyright holders nor the names of the contributors may be used to endorse or promote
    products derived from this software without specific prior written permission.
This software is provided by the copyright holders and contributors “as is” and any express or implied warranties,
 including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose are
 disclaimed. In no event shall copyright holders or contributors be liable for any direct, indirect, incidental,
 special, exemplary, or consequential damages (including, but not limited to, procurement of substitute goods or
 services; loss of use, data, or profits; or business interruption) however caused and on any theory of liability,
 whether in contract, strict liability, or tort (including negligence or otherwise) arising in any way out of the use
 of this software, even if advised of the possibility of such damage.

*/

#include <fstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace dnn;
using namespace std;

// Initialize the parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 256;  // Width of network's input image
int inpHeight = 256; // Height of network's input image
// Give the configuration and weight files for the model
vector<string> classes;

vector<String> getOutputsNames(const Net &net) {
    /*
     * Remove the bounding boxes with low confidence using non-maxima suppression and get the names of the output layers
     */
    static vector<String> names;
    if (names.empty()) {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame) {
    /*
     * Draw the predicted bounding box and draw the predicted bounding box
     */
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty()) {
        CV_Assert(classId < (int) classes.size());
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5 * labelSize.height)),
              Point(left + round(1.5 * labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}

void postProcess(Mat &frame, const vector<Mat> &outs, Rect &detected, bool view = false, float boxXScl = 0.25,
                 float boxYScl = 0.25) {
    /*
     * Remove the bounding boxes with low confidence using non-maxima suppression
     */
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (const auto &out : outs) {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        auto *data = (float *) out.data;
        for (int j = 0; j < out.rows; ++j, data += out.cols) {
            Mat scores = out.row(j).colRange(5, out.cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, nullptr, &confidence, nullptr, &classIdPoint);
            if (confidence > confThreshold && classIdPoint.x == 0) {
                int centerX = (int) (data[0] * (float) frame.cols);
                int centerY = (int) (data[1] * (float) frame.rows);
                int width = (int) (data[2] * (float) frame.cols);
                int height = (int) (data[3] * (float) frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float) confidence);
                boxes.emplace_back(left, top, width, height);
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with lower confidences
    vector<int> indices;
    if (!boxes.empty()) {
        NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
        double maxConfidence = 0;
        int maxIdx = 0;
        for (int idx : indices) {
            if (confidences[idx] > maxConfidence) {
                maxIdx = idx;
                maxConfidence = confidences[idx];
            }
        }
        detected = boxes[maxIdx];
        int oldWidth = detected.width;
        int oldHeight = detected.height;
        detected.width *= boxXScl;
        detected.height *= boxYScl;
        detected.x += (int) (0.5 * (float) (oldWidth - detected.width));
        detected.y += (int) (0.5 * (float) (oldHeight - detected.height));
        if (view) {
            drawPred(classIds[maxIdx], confidences[maxIdx], detected.x, detected.y,
                     detected.x + detected.width, detected.y + detected.height, frame);
        }
    }
}

Rect detect(Mat &frame, Net &net, Mat &blob, Rect &detected, bool view = false) {
    // Create a 4D blob from a frame.
    blobFromImage(frame, blob, 1 / 255.0, Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);
    // Sets the input to the network
    net.setInput(blob);
    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    net.forward(outs, getOutputsNames(net));
    // Remove the bounding boxes with low confidence
    postProcess(frame, outs, detected, view);
}

int main(int argc, char **argv) {
    // Load names of classes
    string classesFile = "coco.names";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    String modelConfiguration = "yolov3.cfg";
    String modelWeights = "yolov3.weights";

    // Load the network
    Net net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

    // Open a video file or an image file or a camera stream.
    VideoCapture cap = VideoCapture(0);
    Mat frame, blob;

    static const string kWinName = "viewfinder";
    namedWindow(kWinName, WINDOW_NORMAL);

    while (waitKey(1) < 0) {
        // get frame from the video
        cap >> frame;
        if (frame.empty()) {
            continue;
        }

        Rect detected;
        detect(frame, net, blob, detected, true);

        // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
//        vector<double> layersTimes;
//        double freq = getTickFrequency() / 1000;
//        double t = net.getPerfProfile(layersTimes) / freq;
//        string label = format("Inference time for a frame : %.2f ms", t);
//        putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

        imshow(kWinName, frame);
        if ((char) waitKey(10) == 'q') {
            break;
        }
    }
    cap.release();
    return 0;
}
