#ifndef FOLLOWBOT_HUMAN_H
#define FOLLOWBOT_HUMAN_H


#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <node.h>

class HumanDetector {
    const float CONF_THRESHOLD = 0.5; // Confidence threshold
    const float XMAX_SUPPRESSION_THRESHOLD = 0.4;  // Non-maximum suppression threshold
    const int FRAME_WIDTH = 256;  // Width of network's input image
    const int FRAME_HEIGHT = 256; // Height of network's input image
    const std::vector<std::string> classes{"person"};
    const cv::String MODEL_CONFIG = "config/yolov3.cfg";
    const cv::String MODEL_WEIGHTS = "config/yolov3.weights";
    const float BOX_X_SCALE = 0.25;
    const float BOX_Y_SCALE = 0.25;
    const float DIST_LIMIT = 20.;

    cv::dnn::Net net;

    std::vector<cv::String> getOutputsNames();
    void postProcess(cv::Mat &frame, const std::vector<cv::Mat> &outs, cv::Rect &detected, bool &foundPerson);
    void detect(Mat &frame, Rect &detected) {

    public:
        void setupNetwork();
        followbot::Point2 getHumanPosition(cv::Mat &rectifiedImg, cv::Mat &pointcloud);
};

#endif //FOLLOWBOT_HUMAN_H
