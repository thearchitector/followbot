#include <astar.hpp>

/*
 * Populate the occupancy grid. innerOccupancy represents the occupancy of the squares that are bounded by the points
 * represented in occupied (where the squares are identified by the location of their top right corner - hence the +1
 * after the floor division); innerOccupancy keeps track of how many points there are within that particular square
 * after the points have been converted into a the scale of the occupancy grid (by dividing by OCCUPANCY_GRID_SCALE). Once
 * a square in innerOccupancy has VOXEL_DENSITY_THRESH points or more, its four bounding corners are added as
 * blocked points to occupied.
 */
void AStar::fillOccupanyGrid(const followbot::WorldConstPtr &world_msg) {
    std::map<IntPair, int> innerOccupancy{};
    std::map<IntPair, bool> alreadyFilledOccupancyAt{};
    #ifndef PRODUCTION
    obugger.clear();
    #endif

    std::map<IntPair, bool> person_bubble{};
    if (person_is_found) {  //. if the person is found, create a 'bubble' around them where no points will be marked as occupied
        for (int i=dest_person_int.first -1; i <= dest_person_int.first + 1; ++i) {
            for (int j=dest_person_int.second -1; j <= dest_person_int.second + 1; ++j) {
                person_bubble.insert({IntPair{i, j}, true});
            }
        }
    }
    occupied.clear();
    for (auto &it : world_msg->buffer) {
        IntPair xzIntPair = IntPair{(int) floor(it.x / OCCUPANCY_GRID_SCALE), (int) floor(it.z / OCCUPANCY_GRID_SCALE)};
        auto foundAtXIntYInt = innerOccupancy.find(xzIntPair);
        if (foundAtXIntYInt == innerOccupancy.end()) {
            innerOccupancy.insert({xzIntPair, 1});
        } else {
            foundAtXIntYInt->second += 1; // increment the number of points recorded in this square
            if (foundAtXIntYInt->second >= VOXEL_DENSITY_THRESH &&
                alreadyFilledOccupancyAt.find(xzIntPair) == alreadyFilledOccupancyAt.end()) {
                alreadyFilledOccupancyAt.insert({xzIntPair, true});
                for (int xNew = xzIntPair.first; xNew <= xzIntPair.first + 1; ++xNew) {
                    for (int yNew = xzIntPair.second; yNew <= xzIntPair.second + 1; ++yNew) {
                        if (person_bubble.find(IntPair{xNew, yNew}) == person_bubble.end()) {
                            occupied.insert({IntPair{xNew, yNew}, true});
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
void AStar::showPersonLoc() {
    if (!buggerWindow.wasStopped()) {
        cv::Point3i person_center_grid = {dest_person_int.first, 0, dest_person_int.second};
        if (!obugger.empty()) {
            cv::viz::WCloud grid_widget = cv::viz::WCloud(obugger);
            grid_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);
            buggerWindow.showWidget("Occupancy", grid_widget);
        }

        cv::viz::WCircle person_circle_grid(0.5, person_center_grid, {0, 1, 0}, 0.1, cv::viz::Color::orange_red());
        buggerWindow.showWidget("Person", person_circle_grid);
        buggerWindow.spinOnce(30, true);
    }
}
#endif

void AStar::handlePersonLoc() {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (current_person_int.first == 0 && current_person_int.second == 0) {
        time_since_person_found = (int) std::chrono::duration_cast<std::chrono::milliseconds>(now - timer_start).count();
        if (time_since_person_found > PERSON_LOC_TIMEOUT) {
            person_is_found = false;
            dest_person_int = IntPair{0, 0};
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
//    current_heading = (short)std::round(std::atan2(world_msg->person.x, world_msg->person.z));
    current_person_int = IntPair{floor(world_msg->person.x / OCCUPANCY_GRID_SCALE), floor(world_msg->person.z / OCCUPANCY_GRID_SCALE)};
    path.clear();

    handlePersonLoc();
    if (person_is_found) {
        fillOccupanyGrid(world_msg);
        #ifndef PRODUCTION
        showPersonLoc();
        #endif
        findAStarPath();
    } else {
        #ifndef PRODUCTION
        fillOccupanyGrid(world_msg);
        showPersonLoc();
        #endif
        // TODO: change path to (0, 0)
    }
    current_heading = (short) (180 * atan2((double)dest_person_int.second, (double)dest_person_int.first) / PI);
    std::cout << "Current heading: " << current_heading << " degrees" << std::endl;
}

bool AStar::isUnBlocked(const IntPair &point) {
    // returns whether a node is unblocked
    auto found = occupied.find(point);
    return found == occupied.end() ? true : !found->second;
}

bool AStar::isDestination(const IntPair &point, const IntPair &dest) {
    return point == dest;
}

float AStar::calculateH(const IntPair &point, const IntPair &dest) {
    return (float) (abs(point.first - dest.second) + abs(point.first - dest.second));
}

// heavily adapted from Andris Jasnsons' implementation here: https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
void AStar::findAStarPath() {
    std::vector<AStarNode> empty;
    // if the src is marked as an obstacle, indicate that there is no obstacle at the source location
    if (occupied.find(ROBOT_POSE) != occupied.end() && occupied.find(ROBOT_POSE)->second) {
        occupied.insert({ ROBOT_POSE, false });
    }

    // check for whether the person's location is at (0, 0)
    if (isDestination(IntPair{0, 0}, dest_person_int)) {
        std::cout << "A*: src == dest " << std::endl;
        path = {AStarNode{dest_person_int.first, dest_person_int.second, ROBOT_POSE.first, ROBOT_POSE.second, 0., 0., 0.}};
        return;
    }

    std::map<IntPair, AStarNode> allMap;
    std::map<IntPair, bool> closedList;
    std::vector<AStarNode> openList;
    int x = 0;
    int y = 0;
    allMap.insert({IntPair(x, y), AStarNode{x, y, x, y, 0., 0., 0.}});
    openList.emplace_back(allMap.find(IntPair(x, y))->second);

    int counter = 0;
    while (!openList.empty() && counter < MAX_ASTAR_LOOPS) {
        AStarNode node{};
        do {
            float temp = FLT_MAX;
            std::vector<AStarNode>::iterator itNode;
            for (auto it = openList.begin();
                 it != openList.end(); it = next(it)) {
                AStarNode n = *it;
                if (n.f < temp) {
                    temp = n.f;
                    itNode = it;
                }
            }
            node = *itNode;
            openList.erase(itNode);
        } while (!isUnBlocked(IntPair{node.x, node.y}));

        x = node.x;
        y = node.y;
        closedList.insert({IntPair{x, y}, true});

        IntPair newIntPair;
        AStarNode newAStarNode{};
        //For each neighbour starting from North-West to South-East
        for (int newX = -1; newX <= 1; ++newX) {
            for (int newY = -1; newY <= 1; ++newY) {
                float gNew, hNew, fNew;
                newIntPair = IntPair{x + newX, y + newY};
                if (isUnBlocked(newIntPair)) {
                    newAStarNode = AStarNode{newIntPair.first, newIntPair.second, x, y, 0., 0., 0.};
                    if (isDestination(IntPair{x + newX, y + newY}, dest_person_int)) {
                        allMap.insert({newIntPair, newAStarNode});
                        makePath(allMap, dest_person_int);
                        return;
                    } else if (!closedList.find(newIntPair)->second || closedList.find(newIntPair) == closedList.end()) {
                        gNew = node.g + 1.f;
                        hNew = calculateH(newIntPair, dest_person_int);
                        fNew = gNew + hNew;
                        // Check if this path is better than the one already present

                        if (allMap.find(newIntPair) == allMap.end()) {
                            allMap.insert(
                                    {newIntPair,
                                     AStarNode{newIntPair.first, newIntPair.second, x, y, FLT_MAX, FLT_MAX, FLT_MAX}});
                        }
                        AStarNode *foundNewAStarNode = &allMap.find(newIntPair)->second;

                        if (foundNewAStarNode->f == FLT_MAX || foundNewAStarNode->f > fNew) {
                            // Update the details of this neighbour node
                            foundNewAStarNode->f = fNew;
                            foundNewAStarNode->g = gNew;
                            foundNewAStarNode->h = hNew;
                            foundNewAStarNode->parent_x = x;
                            foundNewAStarNode->parent_y = y;
                            openList.emplace_back(*foundNewAStarNode);
                        }
                    }
                }
            }
        }
        ++counter;
    }
    std::cout << " A*: Destination not found (with "<< counter << " loops); returning equivalent of src == dst" << std::endl;
    path = {AStarNode{dest_person_int.first, dest_person_int.second, ROBOT_POSE.first, ROBOT_POSE.second, 0., 0., 0.}};
}


void AStar::makePath(std::map<IntPair, AStarNode> &allMap, const IntPair &dest) {
    std::cout << " A*: Found a path - ";
    int x = dest.first;
    int y = dest.second;
    std::stack<AStarNode> intermediate_path;
    while (true) {
        auto foundAtXY = allMap.find(IntPair{x, y});
        if (foundAtXY != allMap.end()) {
            AStarNode nodeFoundAtXY = foundAtXY->second;
            if (!(nodeFoundAtXY.parent_x == x && nodeFoundAtXY.parent_y == y)) {
                intermediate_path.push(allMap.find(IntPair{x, y})->second);
                int tempX = nodeFoundAtXY.parent_x;
                int tempY = nodeFoundAtXY.parent_y;
                x = tempX;
                y = tempY;
            } else {
                break;
            }
        } else {
            break;
        }
    }
    intermediate_path.push(allMap.find(IntPair{x, y})->second);

    // insert found path into path vector
    while (!intermediate_path.empty()) {
        AStarNode top = intermediate_path.top();
        intermediate_path.pop();
        path.emplace_back(top);
        std::cout << "(" << top.x << ", " << top.y << ") ";  // print out path
    }
}
