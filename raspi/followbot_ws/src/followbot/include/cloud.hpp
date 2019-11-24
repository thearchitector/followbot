#ifndef FOLLOWBOT_CLOUD_H
#define FOLLOWBOT_CLOUD_H


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
#include <costmap_converter/ObstacleArrayMsg.h>

class PointCloud {
    const int LEFT_CAMERA_IDX = 1;
    const int RIGHT_CAMERA_IDX = 2;
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

    const int PREFILTER_CAP = 31;
    const int BLOCK_SIZE = 9; // must be + odd int
    const int MIN_DISPARITY = 0;
    const int NUMBER_OF_DISPARITIES = 32;  // must be + int divisible by 16
    const int TEXTURE_THRESHOLD = 10;
    const int UNIQUENESS_THRESHOLD = 15;
    const int SPECKLE_WINDOW_SIZE = 100;
    const int SPECKLE_RANGE = 32;
    const int DISP12_MAX_DEPTH = 1;

    const std::string INTRINSIC_FILENAME = "config/intrinsics.yml";
    const std::string EXTRINSIC_FILENAME = "config/extrinsics.yml";

    cv::VideoCapture capL;
    cv::VideoCapture capR;
    cv::Mat imgLc, imgRc, imgL_, imgR_, mapL1, mapL2, mapR1, mapR2, Q;
    cv::Ptr<cv::StereoBM> bm;

    void serializeDetectedObstacles(std::vector<costmap_converter::ObstacleMsg> &obstacles, cv::Mat &pointcloud);

    public:
        void setupStereoCameras();
        cv::Mat collectPointCloud(cv::Mat &imgL, costmap_converter::ObstacleArrayMsg &msg);
        void releaseCameras();
};

#endif //FOLLOWBOT_CLOUD_H