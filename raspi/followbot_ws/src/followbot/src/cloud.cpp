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

