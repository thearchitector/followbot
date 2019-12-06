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
#include <followbot/Point2.h>
#include <bits/stdc++.h>
#ifndef PRODUCTION
#include <opencv2/viz.hpp>
#endif

// Creating a shortcut for int, int pair type
typedef std::pair<int, int> Pair;

// A structure to hold the neccesary parameters
struct AStarNode {
    int x, y, parent_x, parent_y;
    float f, g, h;
};

inline bool operator < (const AStarNode& lhs, const AStarNode& rhs)
{// overload "<" to put our struct into a set
    return lhs.f < rhs.f;
}


class PointCloud {
    #ifdef PRODUCTION
    static constexpr int LEFT_CAMERA_IDX = 0;
    static constexpr int RIGHT_CAMERA_IDX = 1;
    #else
    static constexpr int LEFT_CAMERA_IDX = 1;
    static constexpr int RIGHT_CAMERA_IDX = 2;
    #endif

    static constexpr int FRAME_WIDTH = 640;
    static constexpr int FRAME_HEIGHT = 480;
    static constexpr float MIDDLE_PROP = 0.2;
    static constexpr float Z_LIMIT = 8;
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
    static constexpr float ROBOT_DIAMETER = 0.5; // meters
    static constexpr int VOXEL_DENSITY_THRESH = 3;

    std::vector<cv::Point2f> buffer;

    #ifndef PRODUCTION
    std::vector<cv::Point3f> buffer3d;
    std::vector<cv::Point3f> obugger;
    cv::viz::Viz3d pcWindow{"Point Cloud"};
    cv::viz::Viz3d buggerWindow{"Occupancy Grid"};
    #endif
    std::map<Pair, bool> occupied;

    const Pair src = {0, 0};

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

    // A Utility Function to check whether the given cell is blocked or not
    bool isUnBlocked(const Pair &point);
    // A Utility Function to check whether destination cell has been reached or not
    static bool isDestination(const Pair &point, const Pair &dest);
    // A Utility Function to calculate the 'h' heuristics.
    static float calculateH(const Pair &point, const Pair &dest);
    static std::vector<AStarNode> makePath(std::map<Pair, AStarNode> &allMap, const Pair &dest);

    public:
        void setupStereoCameras();
        void collectPointCloud(cv::Mat &imgL_remap_3channel, cv::Mat &pointcloud);
        void releaseCameras();

        #ifndef PRODUCTION
        void showPersonLoc(const followbot::Point2 &personLoc);
        #endif

        std::vector<AStarNode> findAStarPath(const Pair &dest);
        void fillOccupanyGrid();
};

#endif //FOLLOWBOT_CLOUD_HPP