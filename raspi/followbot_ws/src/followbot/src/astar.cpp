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
    for (auto &it : world_msg->buffer) {
        IntPair xyIntPair = IntPair{(int) floor(it.x / OCCUPANCY_GRID_SCALE), (int) floor(it.z / OCCUPANCY_GRID_SCALE)};
        auto foundAtXIntYInt = innerOccupancy.find(xyIntPair);
        if (foundAtXIntYInt == innerOccupancy.end()) {
            innerOccupancy.insert({xyIntPair, 1});
        } else {
            foundAtXIntYInt->second += 1;
            if (foundAtXIntYInt->second >= VOXEL_DENSITY_THRESH &&
                alreadyFilledOccupancyAt.find(xyIntPair) == alreadyFilledOccupancyAt.end()) {
                alreadyFilledOccupancyAt.insert({xyIntPair, true});
                for (int xNew = xyIntPair.first; xNew <= xyIntPair.first + 1; ++xNew) {
                    for (int yNew = xyIntPair.second; yNew <= xyIntPair.second + 1; ++yNew) {
                        occupied.insert({IntPair{xNew, yNew}, true });
                        #ifndef PRODUCTION
                        obugger.emplace_back((float)xNew, 0.0f, (float)yNew);
                        #endif
                    }
                }
            }
        }
    }
}

#ifndef PRODUCTION
void AStar::showPersonLoc(const followbot::Point2 person_loc) {
    if (!buggerWindow.wasStopped()) {
        cv::Point3i person_center_grid = {(int)(person_loc.x / OCCUPANCY_GRID_SCALE), 0, (int)(person_loc.z / OCCUPANCY_GRID_SCALE)};
        if (!obugger.empty()) {
            cv::viz::WCloud grid_widget = cv::viz::WCloud(obugger);
            grid_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);
            buggerWindow.showWidget("Occupancy", grid_widget);
        }

        for (int i=0; i < coord_frame.size(); ++i) {
            buggerWindow.showWidget(coord_frame_names[i], coord_frame[i]);
        }
        cv::viz::WCircle person_circle_grid(0.5, person_center_grid, {0, 1, 0}, 0.1, cv::viz::Color::orange_red());
        buggerWindow.showWidget("Person", person_circle_grid);
        buggerWindow.spinOnce(30, true);
    }
}
#endif

void AStar::handleLocNull(const followbot::WorldConstPtr &world_msg) {
    // TODO
}

void AStar::planHeading(const followbot::WorldConstPtr &world_msg) {
    std::cout << "Person: " << world_msg->person.x << ", " << world_msg->person.z << std::endl;
    current_heading = (short)std::round(std::atan2(world_msg->person.x, world_msg->person.z));

    fillOccupanyGrid(world_msg);
    #ifndef PRODUCTION
    showPersonLoc(world_msg->person);
    #endif
}

bool AStar::isUnBlocked(const IntPair &point) {
    // Returns true if the cell is in the occupied map and is not set to false
    auto found = occupied.find(point);
    return found != occupied.end() && found->second;
}

bool AStar::isDestination(const IntPair &point, const IntPair &dest) {
    return point == dest;
}

float AStar::calculateH(const IntPair &point, const IntPair &dest) {
    return (float) (abs(point.first - dest.second) + abs(point.first - dest.second));
}

// based on https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
std::vector<AStarNode> AStar::findAStarPath(const IntPair &dest) {
    std::vector<AStarNode> empty;
    // if the src is marked as an obstacle, indicate that there is no obstacle at the source location
    if (occupied.find(ROBOT_POSE) != occupied.end() && occupied.find(ROBOT_POSE)->second) {
        occupied.insert({ ROBOT_POSE, false });
    }

    if (isDestination(IntPair{0, 0}, dest)) {
        std::cout << "src == dest" << std::endl;
        std::vector<AStarNode> path = {AStarNode{dest.first, dest.second, ROBOT_POSE.first, ROBOT_POSE.second, 0., 0., 0.}};
        return path;
    }

    std::map<IntPair, AStarNode> allMap;
    std::map<IntPair, bool> closedList;
    std::vector<AStarNode> openList;
    int x = 0;
    int y = 0;
    allMap.insert({IntPair(x, y), AStarNode{x, y, x, y, 0., 0., 0.}});
    openList.emplace_back(allMap.find(IntPair(x, y))->second);

    while (!openList.empty()) {
        AStarNode node{};
        do {
            //This do-while loop could be replaced with extracting the first
            //element from a set, but you'd have to make the openList a set.
            //To be completely honest, I don't remember the reason why I do
            //it with a vector, but for now it's still an option, although
            //not as good as a set performance wise.
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
                    if (isDestination(IntPair{x + newX, y + newY}, dest)) {
                        allMap.insert({newIntPair, newAStarNode});
                        return makePath(allMap, dest);

                    } else if (!closedList.find(newIntPair)->second || closedList.find(newIntPair) == closedList.end()) {
                        gNew = node.g + 1.;
                        hNew = calculateH(newIntPair, dest);
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
    }
    std::cout << "Destination not found; returning equivalent of src == dst" << std::endl;
    std::vector<AStarNode> path = {AStarNode{dest.first, dest.second, ROBOT_POSE.first, ROBOT_POSE.second, 0., 0., 0.}};
    return path;
}


std::vector<AStarNode> AStar::makePath(std::map<IntPair, AStarNode> &allMap, const IntPair &dest) {
    std::cout << "Found a path" << std::endl;
    int x = dest.first;
    int y = dest.second;
    std::stack<AStarNode> path;
    std::vector<AStarNode> usablePath;

    while (true) {
        auto foundAtXY = allMap.find(IntPair{x, y});
        if (foundAtXY != allMap.end()) {
            AStarNode nodeFoundAtXY = foundAtXY->second;
            if (!(nodeFoundAtXY.parent_x == x && nodeFoundAtXY.parent_y == y)) {
                path.push(allMap.find(IntPair{x, y})->second);
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
    path.push(allMap.find(IntPair{x, y})->second);
    while (!path.empty()) {
        AStarNode top = path.top();
        path.pop();
        usablePath.emplace_back(top);
    }
    return usablePath;
}
