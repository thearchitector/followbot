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

void PointCloud::filterCloud(std::vector<Point2f> &buffer) {

}

void PointCloud::collectPointCloud(Mat &imgL, Mat &pointcloud, std::vector<cv::Point2f> &buffer) {
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

    filterCloud(buffer);
}

void PointCloud::showPersonLoc(const followbot::Point2 &personLoc, const std::vector<Point2f> &buffer) {
    std::vector<cv::Point3f> buffer3d;

    for (auto &i : buffer) {
        buffer3d.emplace_back(i.x, 0, i.y);
    }

    Point3d person_center = { (double)personLoc.x, 0, (double)personLoc.z };
    viz::WCircle person_circle(0.1, person_center, {0, 1, 0}, 0.01, viz::Color::blue());
    viz::WCloud cloud_widget = viz::WCloud(buffer3d);
    cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);

    if (!myWindow.wasStopped()) {
        myWindow.showWidget("Depth", cloud_widget);
        myWindow.showWidget("person_loc", person_circle);
        myWindow.spinOnce(30, true);
    }
}

void PointCloud::releaseCameras() {
    capL.release();
    capR.release();
}