/* astar.cpp
 * FollowBot POE Project
 * @authors: Duncan Mazza and Elias Gabriel
 *
 * Contains definitions for the AStar class.
 *
 */

#include <astar.hpp>

void AStar::fillOccupanyGrid(const followbot::WorldConstPtr &world_msg) {
    /*
     * Populate the occupancy grid. innerOccupancy represents the occupancy of the squares that are bounded by the
     * points represented in occupied (where the squares are identified by the location of their bottom left right
     * corner). innerOccupancy keeps track of how many points there are within that particular square after the points
     * have been converted into a the scale of the occupancy grid (by dividing by OCCUPANCY_GRID_SCALE). Once a square
     * in innerOccupancy has VOXEL_DENSITY_THRESH points or more, its four bounding corners are added as blocked points
     * to occupied.
     */
    std::map<Node, int> innerOccupancy{};
    std::map<Node, bool> alreadyFilledOccupancyAt{};
    #ifndef PRODUCTION
    obugger.clear();
    #endif

    std::map<Node, bool> person_bubble{};
    if (person_is_found) {  // if the person is found, create a 'bubble' around them where no points will be marked as
        // occupied
        for (int i = dest_person_int.first - 1; i <= dest_person_int.first + 1; ++i) {
            for (int j = dest_person_int.second - 1; j <= dest_person_int.second + 1; ++j) {
                person_bubble.insert({Node{i, j}, true});
            }
        }
    }
    occupied.clear();
    for (auto &it : world_msg->buffer) {
        Node xzNode = Node{(int) floor(it.x / OCCUPANCY_GRID_SCALE), (int) floor(it.z / OCCUPANCY_GRID_SCALE)};
        auto foundAtXIntYInt = innerOccupancy.find(xzNode);
        if (foundAtXIntYInt == innerOccupancy.end()) {
            innerOccupancy.insert({xzNode, 1});
        } else {
            foundAtXIntYInt->second += 1; // increment the number of points recorded in this square
            if (foundAtXIntYInt->second >= VOXEL_DENSITY_THRESH &&
                alreadyFilledOccupancyAt.find(xzNode) == alreadyFilledOccupancyAt.end()) {
                alreadyFilledOccupancyAt.insert({xzNode, true});
                for (int xNew = xzNode.first; xNew <= xzNode.first + 1; ++xNew) {
                    for (int yNew = xzNode.second; yNew <= xzNode.second + 1; ++yNew) {
                        if (person_bubble.find(Node{xNew, yNew}) == person_bubble.end()) {
                            occupied.insert({Node{xNew, yNew}, true});
                            #ifndef PRODUCTION
                            obugger.emplace_back((float) xNew, 0.0f, (float) yNew);
                            #endif
                        }
                    }
                }
            }
        }
    }
}

#ifndef PRODUCTION
void AStar::showPersonPathAndHeading() {
    /*
     * TODO: Documentation
     */
    if (!buggerWindow.wasStopped()) {
        cv::Point3i person_center_grid = {dest_person_int.first, 0, dest_person_int.second};
        // display the occupancy grid
        if (!obugger.empty()) {
            cv::viz::WCloud grid_widget = cv::viz::WCloud(obugger);
            grid_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);
            buggerWindow.showWidget("Occupancy", grid_widget);
        }

        // display the path generated by A* by drawing lines between the path's nodes
        if (!path.empty()) {
            for (int i = 0; i < path.size() - 1; ++i) {
                auto arrow = cv::viz::WCircle(0.2,
                        {static_cast<double>(path[i].first), 0, static_cast<double>(path[i].second)},
                        {0, 1, 0}, 0.1, cv::viz::Color::green());
                buggerWindow.showWidget("line" + std::to_string(i), arrow);
            }
        }

        // draw circle over location of person in occupancy grid
        cv::viz::WCircle person_circle_grid(0.5, person_center_grid, {0, 1, 0}, 0.1, cv::viz::Color::orange_red());
        buggerWindow.showWidget("Person", person_circle_grid);
        buggerWindow.spinOnce(30, true);
        if (!path.empty()) {
            for (int i=0; i < path.size() - 1; ++i) {
                buggerWindow.removeWidget("line" + std::to_string(i));
            }
        }
    }
}
#endif

void AStar::handlePersonLoc() {
    /*
     * Enables the person location to persist for a certain amount of time even if the person isn't found in a given
     * frame
     */
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (current_person_int.first == 0 && current_person_int.second == 0) {
        time_since_person_found = (int) std::chrono::duration_cast<std::chrono::milliseconds>(
                now - timer_start).count();
        if (time_since_person_found > PERSON_LOC_TIMEOUT) {
            person_is_found = false;
            dest_person_int = Node{0, 0};
        } else {
            person_is_found = true;
        }
    } else {
        person_is_found = true;
        timer_start = now;
        time_since_person_found = 0;
        dest_person_int = current_person_int;
    }
    std::cout << "dest person int: " << dest_person_int.first << " " << dest_person_int.second << "  ";
}

void AStar::planHeading(const followbot::WorldConstPtr &world_msg) {
    /*
     * Runs the A* algorithm if a person is found and calculates a heading for the robot to travel at
     */
    current_person_int = Node{floor(world_msg->person.x / OCCUPANCY_GRID_SCALE),
                                 floor(world_msg->person.z / OCCUPANCY_GRID_SCALE)};
    path.clear();

    handlePersonLoc();
    if (person_is_found) {
        fillOccupanyGrid(world_msg);
        findAStarPath();
        #ifndef PRODUCTION
        showPersonPathAndHeading();
        #endif
    } else {
        path = {{ROBOT_POSE.first, ROBOT_POSE.second}};
        #ifndef PRODUCTION
        fillOccupanyGrid(world_msg);
        showPersonPathAndHeading();
        #endif
    }
    current_heading = (short) (180 * atan2((double) path.end()[-2].second, (double) path.end()[-2].first) / PI);
    std::cout << "Current heading: " << current_heading << " degrees" << std::endl;
}

bool AStar::isUnBlocked(const Node &point) {
    /*
     * Returns whether a node is unblocked
     */
    auto found = occupied.find(point);
    if (found == occupied.end()) {
        return true;
    } else {
        return !found->second;
    }
}

bool AStar::isDestination(const Node point) {
    /*
     * Returns whether a point is the destination
     */
    return point == dest_person_int;
}

float AStar::calculateH(const Node &point) {
    /*
     * Calculates a heuristic value using a Manhattan distance metric for a provided point in the occupancy map
     */
    return (float) (abs(point.first - dest_person_int.first) + abs(point.second - dest_person_int.second));
}

void AStar::findAStarPath() {
    /*
     * Runs an implementation of A* to find a path from the center of the occupancy grid to the location of the person
     * in the point cloud (given by the attribute current_person_int). Uses a L1 norm heuristic.
     *
     * note: path attribute is already cleared before this function call
     */
    
    // if the src is marked as an obstacle, indicate that there is no obstacle at the source location
    if (occupied.find(ROBOT_POSE) != occupied.end() && occupied.find(ROBOT_POSE)->second) {
        occupied.insert({ROBOT_POSE, false});
    }

    // check for whether the person's location is at (0, 0)
    if (dest_person_int == ROBOT_POSE) {
        std::cout << "A*: src == dest (here) " << std::endl;
        path = {{ROBOT_POSE.first, ROBOT_POSE.second}};
        return;
    }

    std::map<Node, Node> cameFrom;  // map to store all nodes; when inserting: key = neighbor, value = current
    std::vector<Node> openList;
    // put the starting node in the open list
    openList.emplace_back(Node{ROBOT_POSE.first, ROBOT_POSE.second});
    std::map<Node, float> openMap;  // map for whether a node is in the
    openMap.insert({{ROBOT_POSE.first, ROBOT_POSE.second}, true});
    std::map<Node, float> gMap;  // stores the cheapest known cost of path from (0, 0) to the key Node
    gMap.insert({{ROBOT_POSE.first, ROBOT_POSE.second}, 0});  // initialize gMap at src
    std::map<Node, float> fMap;  // stores the f-cost at each keyed location
    fMap.insert({{ROBOT_POSE.first, ROBOT_POSE.second}, 0});  // initialize fMap at src

    int x = ROBOT_POSE.first;
    int z = ROBOT_POSE.second;
    int counter = 0;
    while (!openList.empty() && counter < MAX_ASTAR_LOOPS) {
        // find the unblocked node with the lowest f in the open list and pop it off the open list
        Node currentNode{};
        float temp_f = FLT_MAX;
        std::vector<Node>::iterator itNode;
        for (auto it = openList.begin(); it != openList.end(); it = next(it)) {
            auto at_it = fMap.find(Node{it->first, it->second});
            float f_at_it = at_it == fMap.end() ? FLT_MAX : at_it->second;
            if (f_at_it < temp_f && isUnBlocked(Node{it->first, it->second})) {
                temp_f = f_at_it;
                itNode = it;
            }
        }
        currentNode = *itNode;
        openList.erase(itNode);  // remove current node form the openList
        openMap.insert({{itNode->first, itNode->second}, false});  // mark the current node as not open in the
        // openMap

        x = currentNode.first;
        z = currentNode.second;
        float g_at_current = gMap.find({x, z})->second;
        Node newNode;
        for (int deltaX = -1; deltaX <= 1; ++deltaX) {  // loop through each adjacent node
            for (int deltaY = -1; deltaY <= 1; ++deltaY) {
                newNode = Node{x + deltaX, z + deltaY};
                if (newNode != currentNode) {
                    if (isUnBlocked(newNode)) {  // if the adjacent node is unblocked
                        // create an AStarNode object out of the node
                        if (isDestination(newNode)) {
                            // terminate algorithm if the adjacent node is the destination
                            cameFrom.insert({newNode, currentNode});
                            makePath(cameFrom);
                            return;
                        } else {
                            float g = gMap.find({currentNode.first, currentNode.second})->second;
                            float gNew = g + (deltaX == 0 || deltaY == 0 ? 1.f : SQRT_2);  // calculate step cost
                            auto g_at_neighbor = gMap.find(newNode);
                            if (g_at_neighbor == gMap.end()) {  // this is the best path
                                // seen yet - record it
                                cameFrom.insert({newNode, currentNode});
                                gMap.insert({newNode, gNew});  // update g at newNode
                                fMap.insert({{newNode.first, newNode.second}, gNew + calculateH(newNode)});
                            } else {
                                if (gNew < g_at_neighbor->second) {  // this is the best path
                                    // seen yet - record it
                                    cameFrom.insert({newNode, currentNode});
                                    gMap.insert({newNode, gNew});  // update g at newNode
                                    fMap.insert({{newNode.first, newNode.second}, gNew + calculateH(newNode)});
                                }
                            }
                            auto open_map_at_neighbor = openMap.find(newNode);
                            if (open_map_at_neighbor == openMap.end()) {
                                openMap.insert({newNode, true});
                                openList.push_back(newNode);
                            } else {
                                if (!open_map_at_neighbor->second) {
                                    openMap.insert({newNode, true});
                                    openList.push_back(newNode);
                                }
                            }
                        }
                    }
                }
            }
        }
        ++counter;
    }
    std::cout << " A*: Destination not found (with " << counter << " loops); returning equivalent of src == dst" << std::endl;
    path = {{ROBOT_POSE.first, ROBOT_POSE.second}};
}


void AStar::makePath(std::map<Node, Node> &comeFrom) {
    /*
     * Constructs the A* path when the A* method finds the destination
     */
    std::cout << " A*: Found a path - ";
    int x = dest_person_int.first;
    int z = dest_person_int.second;
    path.clear();
    while (true) {
        std::cout << "(" << x << ", " << z << ") ";  // print out path
        if (x == ROBOT_POSE.first && z == ROBOT_POSE.second) {
            path.emplace_back(ROBOT_POSE.first, ROBOT_POSE.second);
            return;
        }
        auto foundAtXY = comeFrom.find(Node{x, z});
        path.push_back(foundAtXY->first);
        x = foundAtXY->second.first;
        z = foundAtXY->second.second;
    }
}
