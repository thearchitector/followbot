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
    capR.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capR.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

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
    Mat K1, D1, K2, D2;
    fs["K1"] >> K1;
    fs["D1"] >> D1;
    fs["K2"] >> K2;
    fs["D2"] >> D2;

    fs.open(EXTRINSIC_FILENAME, FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", EXTRINSIC_FILENAME.c_str());
        exit(-1);
    }

    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify(K1, D1, K2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

    bm = StereoBM::create(NUMBER_OF_DISPARITIES, BLOCK_SIZE);
    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(PREFILTER_CAP);
    bm->setBlockSize(BLOCK_SIZE);
    bm->setMinDisparity(MIN_DISPARITY);
    bm->setNumDisparities(NUMBER_OF_DISPARITIES);
    bm->setTextureThreshold(TEXTURE_THRESHOLD);
    bm->setUniquenessRatio(UNIQUENESS_THRESHOLD);
    bm->setSpeckleWindowSize(SPECKLE_WINDOW_SIZE);
    bm->setSpeckleRange(SPECKLE_RANGE);
    bm->setDisp12MaxDiff(DISP12_MAX_DEPTH);

    initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, mapL1, mapL2);
    initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2, mapR1, mapR2);
}

void PointCloud::collectPointCloud(Mat &imgL, Mat &pointcloud) {
    Mat imgR, disp, floatDisp;
    float disparity_multiplier = 1.0f;

    capL.grab();
    capR.grab();
    capL.retrieve(imgLc);
    capR.retrieve(imgRc);

    remap(imgLc, imgL, mapL1, mapL2, INTER_LINEAR);
    remap(imgLc, imgR, mapR1, mapR2, INTER_LINEAR);

    cvtColor(imgL, imgLg, COLOR_BGR2GRAY);
    cvtColor(imgR, imgRg, COLOR_BGR2GRAY);
    bm->compute(imgLg, imgRg, disp);

    if (disp.type() == CV_16S) {
        disparity_multiplier = 16.0f;
    }

    disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
    reprojectImageTo3D(floatDisp, pointcloud, Q, true);

    buffer.clear();
    Point3f xyz_point;

    for (int i = i_min; i < i_max; i++) {
        for (int j = 0; j < pointcloud.cols; j++) {
            xyz_point = pointcloud.at<Point3f>(i, j);
            if (xyz_point.z < Z_LIMIT && xyz_point.y >= Y_RANGE_MIN && xyz_point.y <= Y_RANGE_MAX) {
                buffer.emplace_back(xyz_point.x, xyz_point.z);
            }
        }
    }

    filterCloud();
    fillOccupanyGrid();
}

void PointCloud::releaseCameras() {
    capL.release();
    capR.release();
}

void PointCloud::filterCloud() {
    /*
     * Filter the point cloud
     */
    // TODO
}

void PointCloud::fillOccupanyGrid() {
    /*
     * Populate the occupancy grid. innerOccupancy represents the occupancy of the squares that are bounded by the points
     * represented in occupied (where the squares are identified by the location of their top right corner - hence the +1
     * after the floor division); innerOccupancy keeps track of how many points there are within that particular square
     * after the points have been converted into a the scale of the occupancy grid (by dividing by ROBOT_DIAMETER). Once
     * a square in innerOccupancy has VOXEL_DENSITY_THRESH points or more, its four bounding corners are added as
     * blocked points to occupied.
     */
    std::map<Pair, int> innerOccupancy;
    std::map<Pair, bool> alreadyFilledOccupancyAt;
    for (auto it = buffer.begin(); it != buffer.end(); next(it)) {
        // make integer
        Pair xyPair = Pair{(int) floor(it->x / ROBOT_DIAMETER) + 1, (int) floor(it->x / ROBOT_DIAMETER) + 1};
        auto foundAtXIntYInt = innerOccupancy.find(xyPair);
        if (foundAtXIntYInt == innerOccupancy.end()) {
            innerOccupancy.insert({xyPair, 1});
        } else {
            foundAtXIntYInt->second += 1;
            if (foundAtXIntYInt->second >= VOXEL_DENSITY_THRESH &&
                alreadyFilledOccupancyAt.find(xyPair) != alreadyFilledOccupancyAt.end()) {
                alreadyFilledOccupancyAt.insert({xyPair, true});
                for (int xNew = xyPair.first; xNew >= xyPair.first - 1; xNew--) {
                    for (int yNew = xyPair.first; yNew >= xyPair.first - 1; yNew--) {
                        occupied.insert({Pair{xNew, yNew}, true});
                    }
                }
            }
        }
    }
}

void PointCloud::showPersonLoc(const followbot::Point2 &personLoc) {
    std::vector<cv::Point3f> buffer3d;
    for (auto &i : buffer) {
        buffer3d.emplace_back(i.x, 0, i.y);
    }
    Point3d person_center = {(double) personLoc.x, 0, (double) personLoc.z};
    std::cout << "visualize here; person center = " << person_center << std::endl;

    // TODO: Visualization
//    const Vec3d circle_norm = {0, 1, 0};
//    viz::WCircle person_circle(0.1, person_center, circle_norm, viz::Color::blue());
//    viz::WCloud cloud_widget = viz::WCloud(buffer3d);
//    cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);
//    if (!myWindow.wasStopped()) {
//        myWindow.showWidget("Depth", cloud_widget);
//        myWindow.showWidget("person_loc", person_circle);
//        myWindow.spinOnce(30, true);
//    } else {
//        exit(-1);
//    }
}


bool PointCloud::isUnBlocked(const Pair &point) {
    // Returns true if the cell is in the occupied map and is not set to false
    auto found = occupied.find(point);
    return found != occupied.end() && found->second;
}

bool PointCloud::isDestination(const Pair &point, const Pair &dest) {
    return point == dest;
}


float PointCloud::calculateH(const Pair &point, const Pair &dest) {
    return (float) (abs(point.first - dest.second) + abs(point.first - dest.second));
}

// based on https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
std::vector<AStarNode> PointCloud::findAStarPath(const Pair &dest) {
    std::vector<AStarNode> empty;
    // if the src is marked as an obstacle, indicate that there is no obstacle at the source location
    if (occupied.find(src) != occupied.end() && occupied.find(src)->second) {
        occupied.insert({src, false});
    }

    if (isDestination(Pair{0, 0}, dest)) {
        std::cout << "src == dest" << std::endl;
        std::vector<AStarNode> path = {AStarNode{dest.first, dest.second, src.first, src.second, 0., 0., 0.}};
        return path;
    }

    std::map<Pair, AStarNode> allMap;
    std::map<Pair, bool> closedList;
    std::vector<AStarNode> openList;
    int x = 0;
    int y = 0;
    allMap.insert({Pair(x, y), AStarNode{x, y, x, y, 0., 0., 0.}});
    openList.emplace_back(allMap.find(Pair(x, y))->second);

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
        } while (!isUnBlocked(Pair{node.x, node.y}));

        x = node.x;
        y = node.y;
        closedList.insert({Pair{x, y}, true});

        Pair newPair;
        AStarNode newAStarNode{};
        //For each neighbour starting from North-West to South-East
        for (int newX = -1; newX <= 1; newX++) {
            for (int newY = -1; newY <= 1; newY++) {
                float gNew, hNew, fNew;
                newPair = Pair{x + newX, y + newY};
                if (isUnBlocked(newPair)) {
                    newAStarNode = AStarNode{newPair.first, newPair.second, x, y, 0., 0., 0.};
                    if (isDestination(Pair{x + newX, y + newY}, dest)) {
                        allMap.insert({newPair, newAStarNode});
                        return makePath(allMap, dest);

                    } else if (!closedList.find(newPair)->second || closedList.find(newPair) == closedList.end()) {
                        gNew = node.g + 1.;
                        hNew = calculateH(newPair, dest);
                        fNew = gNew + hNew;
                        // Check if this path is better than the one already present

                        if (allMap.find(newPair) == allMap.end()) {
                            allMap.insert(
                                    {newPair,
                                     AStarNode{newPair.first, newPair.second, x, y, FLT_MAX, FLT_MAX, FLT_MAX}});
                        }
                        AStarNode *foundNewAStarNode = &allMap.find(newPair)->second;

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
    std::vector<AStarNode> path = {AStarNode{dest.first, dest.second, src.first, src.second, 0., 0., 0.}};
    return path;
}


std::vector<AStarNode> PointCloud::makePath(std::map<Pair, AStarNode> &allMap, const Pair &dest) {
    try {
        std::cout << "Found a path" << std::endl;
        int x = dest.first;
        int y = dest.second;
        std::stack<AStarNode> path;
        std::vector<AStarNode> usablePath;

        while (true) {
            auto foundAtXY = allMap.find(Pair{x, y});
            if (foundAtXY != allMap.end()) {
                AStarNode nodeFoundAtXY = foundAtXY->second;
                if (!(nodeFoundAtXY.parent_x == x && nodeFoundAtXY.parent_y == y)) {
                    path.push(allMap.find(Pair{x, y})->second);
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
        path.push(allMap.find(Pair{x, y})->second);
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
