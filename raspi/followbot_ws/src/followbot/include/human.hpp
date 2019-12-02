#ifndef FOLLOWBOT_HUMAN_HPP
#define FOLLOWBOT_HUMAN_HPP


#include <fstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <followbot/Point2.h>


class HumanDetector {
    static constexpr float CONF_THRESHOLD = 0.5; // Confidence threshold
    static constexpr float XMAX_SUPPRESSION_THRESHOLD = 0.4;  // Non-maximum suppression threshold
    static constexpr int FRAME_WIDTH = 256;  // Width of network's input image
    static constexpr int FRAME_HEIGHT = 256; // Height of network's input image
    static constexpr float BOX_X_SCALE = 0.25;
    static constexpr float BOX_Y_SCALE = 0.25;
    static constexpr float DIST_LIMIT = 20;
    const cv::String MODEL_CONFIG = "config/yolov3.cfg";
    const cv::String MODEL_WEIGHTS = "config/yolov3.weights";

    cv::dnn::Net net;

    std::vector<cv::String> getOutputsNames();
    static bool postProcess(cv::Mat &frame, const std::vector<cv::Mat> &outs, cv::Rect &detected, bool view);
    bool detect(cv::Mat &frame, cv::Rect &detected);
    static void drawPred(float conf, int left, int top, int right, int bottom, cv::Mat &frame);

    public:
        void setupNetwork();
        followbot::Point2 getHumanPosition(cv::Mat &rectifiedImg, cv::Mat &pointcloud, followbot::Point2 &human_loc);
};

#endif //FOLLOWBOT_HUMAN_HPP
