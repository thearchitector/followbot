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
{// overload "<" to put our struct into a set
    return lhs.f < rhs.f;
}


class PointCloud {
    // NOTE: ADJUST THESE VALUES TO YOUR SPECIFIC SETUP SUCH THAT THEY ARE CORRECTLY SELECTING THE STEREO CAMERAS
    #ifdef PRODUCTION
    static constexpr int LEFT_CAMERA_IDX = 0;
    static constexpr int RIGHT_CAMERA_IDX = 2;
    #else
    static constexpr int LEFT_CAMERA_IDX = 0;
    static constexpr int RIGHT_CAMERA_IDX = 1;
    #endif

    // Force cameras to read 640x480 frames
    static constexpr int FRAME_WIDTH = 640;
    static constexpr int FRAME_HEIGHT = 480;

    // Parameters for selecting the desired subset of points from the raw 3D point cloud to create the 2D point cloud
    static constexpr float MIDDLE_PROP = 0.2;
    static constexpr float Z_LIMIT = 8;
    static constexpr float Y_RANGE_MIN = -0.2;
    static constexpr float Y_RANGE_MAX = 0.2;
    int middle = std::floor(FRAME_HEIGHT / 2);
    int height_delta = std::floor(FRAME_HEIGHT * (MIDDLE_PROP / 2));
    int i_min = middle - height_delta;
    int i_max = middle + height_delta;

    // Objects for image capture
    cv::VideoCapture capL;
    cv::VideoCapture capR;
    cv::Mat imgLc, imgRc, imgLg, imgRg, mapL1, mapL2, mapR1, mapR2, Q;
    cv::Ptr<cv::StereoBM> bm;

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

    // Parameter for quantizing the point cloud into an occupancy grid of OCCUPANCY_GRID_SCALE x OCCUPANCY_GRID_SCALE
    // squares
    static constexpr float OCCUPANCY_GRID_SCALE = 0.5; // meters
    static constexpr int VOXEL_DENSITY_THRESH = 3;  // minimum number of points required in square in the point cloud
    // occupancy grid to constitute that square as occupied

    // Buffer to store the 2D point cloud
    std::vector<cv::Point2f> buffer;

    #ifndef PRODUCTION
    // Buffer to store the 2D point cloud in a 3D point cloud format so that it can be displayed with viz::WCloud
    std::vector<cv::Point3f> buffer3d;
    cv::viz::Viz3d pcWindow{"Point Cloud"};
    // Buffer to store the 2D occupancy grid vertices in a 3D point cloud format so that it can be displayed with
    // viz::WCloud
    std::vector<cv::Point3f> obugger;
    cv::viz::Viz3d buggerWindow{"Occupancy Grid"};

    std::vector<cv::viz::WLine> coord_frame = {cv::viz::WLine({0, 0, 0}, {1, 0, 0}),
                                               cv::viz::WLine({0, 0, 0}, {0, 1, 0}),
                                               cv::viz::WLine({0, 0, 0}, {0, 0, 1})};
    std::vector<std::string> coord_frame_names = {std::string("ihat"), std::string("jhat"), std::string("khat")};
    #endif

    std::map<IntPair, bool> occupied;  // map of the occupancy grid

    const IntPair src = {0, 0};  // location of the robot in the occupancy grid

    // A Utility Function to check whether the given cell is blocked or not
    bool isUnBlocked(const IntPair &point);
    // A Utility Function to check whether destination cell has been reached or not
    static bool isDestination(const IntPair &point, const IntPair &dest);
    // A Utility Function to calculate the 'h' heuristics.
    static float calculateH(const IntPair &point, const IntPair &dest);
    static std::vector<AStarNode> makePath(std::map<IntPair, AStarNode> &allMap, const IntPair &dest);

    public:
        void setupStereoCameras();
        void collectPointCloud(cv::Mat &imgL_remap_3channel, cv::Mat &pointcloud);
        void releaseCameras();

        #ifndef PRODUCTION
        void showPersonLoc(const followbot::Point2 &personLoc);
        #endif

        std::vector<AStarNode> findAStarPath(const IntPair &dest);
        void fillOccupanyGrid();
};

#endif //FOLLOWBOT_CLOUD_HPP
