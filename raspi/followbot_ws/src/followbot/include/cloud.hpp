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
typedef std::pair<int, int> IntPair;

// A structure to hold the neccesary parameters for a node in the A* algorithm
struct AStarNode {
    int x, y, parent_x, parent_y;
    float f, g, h;
};

inline bool operator < (const AStarNode& lhs, const AStarNode& rhs)
{
    return lhs.f < rhs.f;
}


class PointCloud {
    // NOTE: ADJUST THESE VALUES TO YOUR SPECIFIC SETUP SUCH THAT THEY ARE CORRECTLY SELECTING THE STEREO CAMERAS
    #ifdef PRODUCTION
    static constexpr int LEFT_CAMERA_IDX = 0;
    static constexpr int RIGHT_CAMERA_IDX = 2;
    #else
    static constexpr int LEFT_CAMERA_IDX = 0;
    static constexpr int RIGHT_CAMERA_IDX = 2;
    #endif

    // Force cameras to read 640x480 frames
    static constexpr int FRAME_WIDTH = 640;
    static constexpr int FRAME_HEIGHT = 480;

    // Parameters for selecting the desired subset of points from the raw 3D point cloud to create the 2D point cloud
    static constexpr float MIDDLE_PROP = 0.2;
    static constexpr float Z_LIMIT = 8;
    static constexpr float Y_RANGE_MIN = -0.2;
    static constexpr float Y_RANGE_MAX = 0.2;
    static constexpr int MIDDLE = FRAME_HEIGHT / 2;
    static constexpr int Y_DELTA = FRAME_HEIGHT * (MIDDLE_PROP / 2);
    static constexpr int I_MIN = MIDDLE - Y_DELTA;
    static constexpr int I_MAX = MIDDLE + Y_DELTA;

    // Parameters for the StereoBM object
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

    #ifndef PRODUCTION
    // Buffer to store the 2D point cloud in a 3D point cloud format so that it can be displayed with viz::WCloud
    std::vector<cv::Point3f> buffer3d;
    cv::viz::Viz3d pcWindow{"Point Cloud"};
    // Buffer to store the 2D occupancy grid vertices in a 3D point cloud format so that it can be displayed with
    // viz::WCloud
    std::vector<cv::Point3f> obugger;
    cv::viz::Viz3d buggerWindow{"Occupancy Grid"};
    const std::vector<cv::viz::WLine> coord_frame = {
            cv::viz::WLine({0, 0, 0}, {1, 0, 0}),
            cv::viz::WLine({0, 0, 0}, {0, 1, 0}),
            cv::viz::WLine({0, 0, 0}, {0, 0, 1})
    };
    const std::vector<std::string> coord_frame_names = { "ihat", "jhat", "khat" };
    #endif

    // Parameter for quantizing the point cloud into an OCCUPANCY_GRID_SCALE x OCCUPANCY_GRID_SCALE occupancy grid
    static constexpr float OCCUPANCY_GRID_SCALE = 0.5; // meters
    // minimum number of points required in a grid square to determine occupancy
    static constexpr int VOXEL_DENSITY_THRESH = 3;
    const IntPair ROBOT_POSE = {0, 0};  // location of the robot in the occupancy grid

    // Objects for image capture
    cv::VideoCapture capL;
    cv::VideoCapture capR;
    cv::Mat imgLc, imgRc, imgLg, imgRg, mapL1, mapL2, mapR1, mapR2, Q;
    cv::Ptr<cv::StereoBM> bm;
    // Buffer to store the 2D point cloud
    std::vector<cv::Point2f> buffer;
    std::map<IntPair, bool> occupied;  // map of the occupancy grid

    public:
        void setupStereoCameras();
        void collectPointCloud(cv::Mat &imgL_remap_3channel, cv::Mat &pointcloud);
        void releaseCameras();


};

#endif //FOLLOWBOT_CLOUD_HPP
