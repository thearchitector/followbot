#ifndef ASTAR_HPP
#define ASTAR_HPP


#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <followbot/World.h>
#include <bits/stdc++.h>

#ifndef PRODUCTION
#include <opencv2/viz.hpp>
#endif

// Creating a shortcut for int, int pair type
typedef std::pair<int, int> Node;

class AStar {
    bool isUnBlocked(const Node &point);
    bool isDestination(Node point);
    float calculateH(const Node &point);
    void makePath(std::map<Node, Node> &comeFrom);

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
    static constexpr int VOXEL_DENSITY_THRESH = 10;

    const Node ROBOT_POSE = {0, 0};  // location of the robot in the occupancy grid
    std::map<Node, bool> occupied;  // map of the occupancy grid

    Node current_person_int = Node{0, 0};
    Node dest_person_int = Node{0, 0};
    Node astar_person_int = Node{0, 0};
    bool person_is_found = false;
    int time_since_person_found = 0;
    std::chrono::steady_clock::time_point timer_start = std::chrono::steady_clock::now();
    static constexpr int PERSON_LOC_TIMEOUT = 3000;  // ms to wait before setting person location to 0, 0
    static constexpr float PI = 3.1415;
    static constexpr int MAX_ASTAR_LOOPS = 30;
    std::vector<Node> path = {{ROBOT_POSE.first, ROBOT_POSE.second}};
    static constexpr float SQRT_2 = 1.414f;

    void fillOccupanyGrid(const followbot::WorldConstPtr &world_msg);
    void handlePersonLoc();

    public:
        short current_heading = 0;

        void planHeading(const followbot::WorldConstPtr &world_msg);
        void findAStarPath();

        #ifndef PRODUCTION
        AStar() {
            // add coordinate frame
            for (int i = 0; i < coord_frame.size(); ++i) {
                buggerWindow.showWidget(coord_frame_names[i], coord_frame[i]);
            }
        }
        void showPersonPathAndHeading();
        #endif
};


#endif // ASTAR_HPP