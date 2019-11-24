#include <cloud.hpp>

using namespace cv;

void PointCloud::setupStereoCameras() {
    capL = cv::VideoCapture(0);
    capR = cv::VideoCapture(2);

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
    cvtColor(imgLc, imgL_, COLOR_BGR2GRAY);
    cvtColor(imgRc, imgR_, COLOR_BGR2GRAY);

    Size img_size = imgL_.size();
    if (img_size != imgR_.size()) {
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

    bm = StereoBM::create(16, 9);
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

    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, mapL1, mapL2);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, mapR1, mapR2);
}

void PointCloud::serializeDetectedObstacles(std::vector<costmap_converter::ObstacleMsg> &obstacles, Mat &pointcloud) {

}

Mat PointCloud::collectPointCloud(Mat &imgL, costmap_converter::ObstacleArrayMsg &obstacle_msg) {
    Mat imgR, disp, floatDisp, pointcloud;
    float disparity_multiplier = 1.0f;

    capL.grab();
    capR.grab();
    capL.retrieve(imgLc);
    capR.retrieve(imgRc);
    cvtColor(imgLc, imgL_, COLOR_BGR2GRAY);
    cvtColor(imgRc, imgR_, COLOR_BGR2GRAY);

    remap(imgL_, imgL, mapL1, mapL2, INTER_LINEAR);
    remap(imgR_, imgR, mapR1, mapR2, INTER_LINEAR);

    bm->compute(imgL, imgR, disp);

    if (disp.type() == CV_16S) {
        disparity_multiplier = 16.0f;
    }

    disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
    reprojectImageTo3D(floatDisp, pointcloud, Q, true);

    obstacle_msg.header.stamp = ros::Time::now();
    serializeDetectedObstacles(obstacle_msg.obstacles, pointcloud);

    return pointcloud;
}

void PointCloud::releaseCameras() {
    capL.release();
    capR.release();
}