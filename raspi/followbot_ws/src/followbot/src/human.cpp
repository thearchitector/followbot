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
*/


#include <human.hpp>

using namespace std;
using namespace cv;
using namespace dnn;

void HumanDetector::setupNetwork() {
    net = readNetFromDarknet(MODEL_CONFIG, MODEL_WEIGHTS);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
}

/*
 * Remove the bounding boxes with low confidence using non-maxima suppression and get the names of the output layers
 */
vector<String> HumanDetector::getOutputsNames() {
    static vector<String> names;

    if (names.empty()) {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();
        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i) names[i] = layersNames[outLayers[i] - 1];
    }

    return names;
}

/*
 * Remove the bounding boxes with low confidence using non-maxima suppression
 */
bool HumanDetector::postProcess(cv::Mat &frame, const std::vector<cv::Mat> &outs, cv::Rect &detected) {
    vector<float> confidences;
    vector<Rect> boxes;

    for (const auto &out : outs) {
        // Scan through all the bounding boxes output from the network and keep only the ones with high confidence
        // scores. Assign the box's class label as the class with the highest score for the box.
        auto *data = (float *) out.data;
        for (int j = 0; j < out.rows; ++j, data += out.cols) {
            Mat scores = out.row(j).colRange(5, out.cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, nullptr, &confidence, nullptr, &classIdPoint);
            if (confidence > CONF_THRESHOLD && classIdPoint.x == 0) {  // filter to only detect people
                int centerX = (int) (data[0] * (float) frame.cols);
                int centerY = (int) (data[1] * (float) frame.rows);
                int width = (int) (data[2] * (float) frame.cols);
                int height = (int) (data[3] * (float) frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                confidences.push_back((float) confidence);
                boxes.emplace_back(left, top, width, height);
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with lower confidences
    vector<int> indices;
    if (!boxes.empty()) {
        NMSBoxes(boxes, confidences, CONF_THRESHOLD, XMAX_SUPPRESSION_THRESHOLD, indices);
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
        detected.width *= BOX_X_SCALE;
        detected.height *= BOX_Y_SCALE;
        detected.x += (int) (0.5 * (float) (oldWidth - detected.width));
        detected.y += (int) (0.5 * (float) (oldHeight - detected.height));

        #ifndef PRODUCTION
        drawPred(confidences[maxIdx], detected.x, detected.y, detected.x + detected.width, detected.y + detected.height, frame);
        imshow("Person Detection", frame);
        waitKey(1);
        #endif

        return true;
    }
    return false;
}

bool HumanDetector::detect(Mat &frame, Rect &detected) {
    Mat blob;
    // Create a 4D blob from a frame.
    blobFromImage(frame, blob, 1 / 255.0, Size(FRAME_WIDTH, FRAME_HEIGHT), Scalar(0, 0, 0), true, false);
    // Sets the input to the network
    net.setInput(blob);
    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    net.forward(outs, getOutputsNames());
    // Remove the bounding boxes with low confidence
    return postProcess(frame, outs, detected);
}

void HumanDetector::getHumanPosition(Mat &rectifiedImg, Mat &pointcloud, followbot::World &world_msg) {
    Rect detected;
    float xSum = 0;
    float zSum = 0;
    float count = 1.0;

    if (detect(rectifiedImg, detected)) {
        Point3f xyz_;
        for (int row = detected.y; row <= detected.y + detected.height; row++) {
            for (int col = detected.x; col <= detected.x + detected.width; col++) {
                xyz_ = pointcloud.at<Point3f>(row, col);
                if (xyz_.z <= DIST_LIMIT) {
                    xSum += xyz_.x;
                    zSum += xyz_.z;
                    count++;
                }
            }
        }
    }

    // set the location of the person to the average (x, z) location of the points in the floor plane
    // (given by the x and z axes in the point cloud)
    world_msg.person.x = xSum / count;
    world_msg.person.z = zSum / count;
}

void HumanDetector::drawPred(float conf, int left, int top, int right, int bottom, Mat &frame) {
    /*
     * Draw the predicted bounding box and draw the predicted bounding box
     */
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    label = "person";

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5 * labelSize.height)),
              Point(left + round(1.5 * labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}
