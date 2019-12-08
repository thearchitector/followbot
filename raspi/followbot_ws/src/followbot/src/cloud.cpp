#include <cloud.hpp>


using namespace cv;

void PointCloud::setupStereoCameras() {
    capL = cv::VideoCapture(LEFT_CAMERA_IDX);
    capR = cv::VideoCapture(RIGHT_CAMERA_IDX);

    if (!capL.isOpened() || !capR.isOpened()) {
        std::cout << "Couldn't open one or both of the cameras" << std::endl;
        exit(1);
    }

    capL.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capL.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    capL.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capL.set(CAP_PROP_BUFFERSIZE, 1);

    capR.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capR.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    capR.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capR.set(CAP_PROP_BUFFERSIZE, 1);

    capL.grab();
    capR.grab();
    capL.retrieve(imgLc);
    capR.retrieve(imgRc);
    cvtColor(imgLc, imgLg, COLOR_BGR2GRAY);
    cvtColor(imgRc, imgRg, COLOR_BGR2GRAY);

    Size img_size = imgLg.size();
    if (img_size != imgRg.size()) {
        std::cout << "Camera inputs are not of the same dimension" << std::endl;
        exit(-1);
    }

    FileStorage fs(INTRINSIC_FILENAME, FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", INTRINSIC_FILENAME.c_str());
        exit(-1);
    }

    Rect roi1, roi2;
    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open(EXTRINSIC_FILENAME, FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", EXTRINSIC_FILENAME.c_str());
        exit(-1);
    }

    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

    bm = StereoBM::create(NUMBER_OF_DISPARITIES, BLOCK_SIZE);
    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(PREFILTER_CAP);
    bm->setMinDisparity(MIN_DISPARITY);
    bm->setTextureThreshold(TEXTURE_THRESHOLD);
    bm->setUniquenessRatio(UNIQUENESS_THRESHOLD);
    bm->setSpeckleWindowSize(SPECKLE_WINDOW_SIZE);
    bm->setSpeckleRange(SPECKLE_RANGE);
    bm->setDisp12MaxDiff(DISP12_MAX_DEPTH);

    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, mapL1, mapL2);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, mapR1, mapR2);
}

void PointCloud::collectPointCloud(Mat &imgL_remap_3channel, Mat &pointcloud) {
    Mat imgR_remap, imgL_remap, disp, floatDisp;

    capL.grab();
    capR.grab();
    capL.retrieve(imgLc);
    capR.retrieve(imgRc);

    cvtColor(imgLc, imgLg, COLOR_BGR2GRAY);
    cvtColor(imgRc, imgRg, COLOR_BGR2GRAY);

    remap(imgLg, imgL_remap, mapL1, mapL2, INTER_LINEAR);
    remap(imgRg, imgR_remap, mapR1, mapR2, INTER_LINEAR);
    remap(imgLc, imgL_remap_3channel, mapL1, mapL2, INTER_NEAREST);

    bm->compute(imgL_remap, imgR_remap, disp);

    #ifndef PRODUCTION
    Mat disp8;
    buffer3d.clear();
    disp.convertTo(disp8, CV_8U, 255.0f / (NUMBER_OF_DISPARITIES * 16.0f));
    imshow("disparity", disp8);
    waitKey(1);
    #endif

    disp.convertTo(floatDisp, CV_32F, 0.0625f);
    reprojectImageTo3D(floatDisp, pointcloud, Q, true);

    buffer.clear();
    Point3f *xyz_point;
    for (int i = I_MIN; i < I_MAX; ++i) {
        for (int j = 0; j < pointcloud.cols; ++j) {
            xyz_point = &pointcloud.at<Point3f>(i, j);
            if (xyz_point->z < Z_LIMIT && xyz_point->y >= Y_RANGE_MIN && xyz_point->y <= Y_RANGE_MAX) {
                buffer.emplace_back(xyz_point->x, xyz_point->z);
                #ifndef PRODUCTION
                buffer3d.emplace_back(xyz_point->x, xyz_point->y, xyz_point->z);
                #endif
            }
        }
    }
    fillOccupanyGrid();
}

void PointCloud::releaseCameras() {
    capL.release();
    capR.release();
}

void PointCloud::fillOccupanyGrid() {
    /*
     * Populate the occupancy grid. innerOccupancy represents the occupancy of the squares that are bounded by the points
     * represented in occupied (where the squares are identified by the location of their top right corner - hence the +1
     * after the floor division); innerOccupancy keeps track of how many points there are within that particular square
     * after the points have been converted into a the scale of the occupancy grid (by dividing by OCCUPANCY_GRID_SCALE). Once
     * a square in innerOccupancy has VOXEL_DENSITY_THRESH points or more, its four bounding corners are added as
     * blocked points to occupied.
     */
    std::map<IntPair, int> innerOccupancy{};
    std::map<IntPair, bool> alreadyFilledOccupancyAt{};

    #ifndef PRODUCTION
    obugger.clear();
    #endif

    for (auto &i : buffer) {
        auto *it = &i;
        IntPair xyIntPair = IntPair{(int) floor(it->x / OCCUPANCY_GRID_SCALE), (int) floor(it->y / OCCUPANCY_GRID_SCALE)};
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
void PointCloud::showPersonLoc(const followbot::Point2 &personLoc) {
    if (!pcWindow.wasStopped()) {
        if (!buffer3d.empty()) {
            viz::WCloud cloud_widget = viz::WCloud(buffer3d);
            cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);
            pcWindow.showWidget("Depth", cloud_widget);
        }

        Point3d person_center = {(double) personLoc.x, 0, (double) personLoc.z};
        viz::WCircle person_circle(0.1, person_center, {0, 1, 0}, 0.01, viz::Color::blue());

        pcWindow.showWidget("person_loc", person_circle);
        pcWindow.spinOnce(30, true);
        for (int i=0; i < coord_frame.size(); ++i) {
            pcWindow.showWidget(coord_frame_names[i], coord_frame[i]);
        }
    }

    if (!buggerWindow.wasStopped()) {
        Point3i person_center_grid = {(int)(personLoc.x / OCCUPANCY_GRID_SCALE), 0, (int)(personLoc.z / OCCUPANCY_GRID_SCALE)};
        if (!obugger.empty()) {
            viz::WCloud grid_widget = viz::WCloud(obugger);
            grid_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);
            buggerWindow.showWidget("Occupancy", grid_widget);
        }
        viz::WCircle person_circle_grid(0.5, person_center_grid, {0, 1, 0}, 0.1, viz::Color::orange_red());
        viz::WLine ihatg = viz::WLine({0, 0, 0}, {1 / OCCUPANCY_GRID_SCALE, 0, 0});
        viz::WLine jhatg = viz::WLine({0, 0, 0}, {0, 1 / OCCUPANCY_GRID_SCALE, 0});
        viz::WLine khatg = viz::WLine({0, 0, 0}, {0, 0, 1 / OCCUPANCY_GRID_SCALE});

        buggerWindow.showWidget("Person", person_circle_grid);
        buggerWindow.spinOnce(30, true);
        buggerWindow.showWidget("ihatg", ihatg);
        buggerWindow.showWidget("jhatg", jhatg);
        buggerWindow.showWidget("khatg", khatg);
    }
}
#endif

bool PointCloud::isUnBlocked(const IntPair &point) {
    // Returns true if the cell is in the occupied map and is not set to false
    auto found = occupied.find(point);
    return found != occupied.end() && found->second;
}

bool PointCloud::isDestination(const IntPair &point, const IntPair &dest) {
    return point == dest;
}


float PointCloud::calculateH(const IntPair &point, const IntPair &dest) {
    return (float) (abs(point.first - dest.second) + abs(point.first - dest.second));
}

// based on https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
std::vector<AStarNode> PointCloud::findAStarPath(const IntPair &dest) {
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


std::vector<AStarNode> PointCloud::makePath(std::map<IntPair, AStarNode> &allMap, const IntPair &dest) {
    try {
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
    catch (const Exception &e) {
        std::cout << e.what() << std::endl;
    }
}
