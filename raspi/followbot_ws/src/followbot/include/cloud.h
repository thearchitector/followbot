#ifndef CLOUD_H
#define CLOUD_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/core_c.h>
#include <math.h>
#include <opencv2/photo.hpp>
#include <cstdio>
#include <iostream>

class PointCloud {
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

    const std::string INTRINSIC_FILENAME = "intrinsics.yml";
    const std::string EXTRINSIC_FILENAME = "extrinsics.yml";
    const int LEFT_CAMERA_IDX = 1;
    const int RIGHT_CAMERA_IDX = 2;
    const cv::Point2f Y_RANGE = cv::Point2f(-0.1, 0.2);
    const float MIDDLE_PROP = 0.4;
    const int Z_LIMIT = 20;
    const int LAMBDA = 8000;
    const float SIGMA = 1.5;


    cv::Mat collectPointCloud();
};

#endif //CLOUD_H