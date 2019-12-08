#ifndef ASTAR_HPP
#define ASTAR_HPP


#include <iostream>
#include <vector>
#include <followbot/Point2.h>
#include <opencv2/core.hpp>
#include <followbot/Buffer.h>
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

class AStar {

    bool isUnBlocked(const IntPair &point);
    static bool isDestination(const IntPair &point, const IntPair &dest);
    static float calculateH(const IntPair &point, const IntPair &dest);
    static std::vector <AStarNode> makePath(std::map<IntPair, AStarNode> &allMap, const IntPair &dest);

    void fillOccupanyGrid(const followbot::Buffer& buffer_msg);

    #ifndef PRODUCTION
    std::vector<cv::Point3f> obugger;
    cv::viz::Viz3d buggerWindow{"Occupancy Grid"};
    const std::vector<cv::viz::WLine> coord_frame = {
            cv::viz::WLine({0, 0, 0}, {1 / OCCUPANCY_GRID_SCALE, 0, 0}),
            cv::viz::WLine({0, 0, 0}, {0, 1 / OCCUPANCY_GRID_SCALE, 0}),
            cv::viz::WLine({0, 0, 0}, {0, 0, 1 / OCCUPANCY_GRID_SCALE})
    };
    const std::vector<std::string> coord_frame_names = { "ihatg", "jhatg", "khatg" };
    #endif

    // Parameter for quantizing the point cloud into an OCCUPANCY_GRID_SCALE x OCCUPANCY_GRID_SCALE occupancy grid
    static constexpr float OCCUPANCY_GRID_SCALE = 0.5; // meters
    // minimum number of points required in a grid square to determine occupancy
    static constexpr int VOXEL_DENSITY_THRESH = 3;

    const IntPair ROBOT_POSE = {0, 0};  // location of the robot in the occupancy grid
    std::map<IntPair, bool> occupied;  // map of the occupancy grid

    public:
        std::vector <AStarNode> findAStarPath(const IntPair &dest);
        #ifndef PRODUCTION
        void showPersonLoc(const followbot::Point2 &personLoc);
        #endif
};


#endif // ASTAR_HPP