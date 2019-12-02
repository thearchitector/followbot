#ifndef FOLLOWBOT_CLOUD_HPP
#define FOLLOWBOT_CLOUD_HPP


#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/core_c.h>
#include <cmath>
#include <opencv2/photo.hpp>
#include <cstdio>
#include <iostream>

class PointCloud {
    static constexpr int LEFT_CAMERA_IDX = 1;
    static constexpr int RIGHT_CAMERA_IDX = 2;
    static constexpr int FRAME_WIDTH = 640;
    static constexpr int FRAME_HEIGHT = 480;
    static constexpr float MIDDLE_PROP = 0.2;
    static constexpr float Z_LIMIT = 20;
    static constexpr float Y_RANGE_MIN = -0.2;
    static constexpr float Y_RANGE_MAX = 0.2;
    static constexpr int PREFILTER_CAP = 31;
    static constexpr int BLOCK_SIZE = 9; // must be + odd int
    static constexpr int MIN_DISPARITY = 0;
    static constexpr int NUMBER_OF_DISPARITIES = 32;  // must be + int divisible by 16
    static constexpr int TEXTURE_THRESHOLD = 10;
    static constexpr int UNIQUENESS_THRESHOLD = 15;
    static constexpr int SPECKLE_WINDOW_SIZE = 100;
    static constexpr int SPECKLE_RANGE = 32;
    static constexpr int DISP12_MAX_DEPTH = 1;

    const cv::String INTRINSIC_FILENAME = "config/intrinsics.yml";
    const cv::String EXTRINSIC_FILENAME = "config/extrinsics.yml";

    cv::VideoCapture capL;
    cv::VideoCapture capR;
    cv::Mat imgLc, imgRc, imgLg, imgRg, mapL1, mapL2, mapR1, mapR2, Q;
    cv::Ptr<cv::StereoBM> bm;

    int middle = std::floor(FRAME_HEIGHT / 2);
    int height_delta = std::floor(FRAME_HEIGHT * (MIDDLE_PROP / 2));
    int i_min = middle - height_delta;
    int i_max = middle + height_delta;

public:
    void setupStereoCameras();

    void collectPointCloud(cv::Mat &imgL, cv::Mat &pointcloud);

    void makePointCloudBuffer(cv::Mat &pointcloud, std::vector<cv::Point2f> &buffer);

    void releaseCameras();
};

#endif //FOLLOWBOT_CLOUD_HPP