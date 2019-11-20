//
// Created by duncan on 11/7/19.
//

/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/viz.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/core_c.h>
#include <math.h>
#include <opencv2/photo.hpp>
#include <cstdio>
#include <iostream>

using namespace cv;

void checkParams(int alg, int numberOfDisparities, int SADWindowSize, float scale) {
    if (alg < 0) {
        printf("Unknown stereo algorithm\n\n");
        exit(-1);
    }
    if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0) {
        printf("The max disparity must be a positive integer divisible by 16\n");
        exit(-1);
    }
    if (scale < 0) {
        printf("The scale factor must be a positive floating-point number\n");
        exit(-1);
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1) {
        printf("The block size must be a positive odd number\n");
        exit(-1);
    }
}

int main(int argc, char **argv) {
    enum {
        STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4
    };

    std::string intrinsic_filename = "intrinsics.yml";
    std::string extrinsic_filename = "extrinsics.yml";
    // PARAMETERS
    const int left_camera_idx = 1;
    const int right_camera_idx = 2;
    Point2f y_range = Point2f(-0.1, 0.2);
    const float middle_prop = 0.4;
    int i_min, i_max;
    int alg = STEREO_BM;
    int SADWindowSize = 9; // must be + odd int
    int numberOfDisparities = 32;  // must be + int divisible by 16
    float scale = 1.;  // must be + float
    checkParams(alg, numberOfDisparities, SADWindowSize, scale);
    const int z_limit = 20;
    const int lambda = 8000;
    const float sigma = 1.5;
    // reading intrinsic parameters
    VideoCapture capL(left_camera_idx);
    VideoCapture capR(right_camera_idx);
    if (!capL.isOpened() || !capR.isOpened()) {
        std::cout << "Couldn't open one or both of the cameras" << std::endl;
    }

    capL.set(CAP_PROP_FRAME_WIDTH, 640);
    capL.set(CAP_PROP_FRAME_HEIGHT, 480);
    capR.set(CAP_PROP_FRAME_WIDTH, 640);
    capR.set(CAP_PROP_FRAME_HEIGHT, 480);

    Mat imgLc;
    Mat imgRc;
    Mat imgL_;
    Mat imgR_;
    Mat imgL;
    Mat imgR;

    capL.grab();
    capR.grab();
    capL.retrieve(imgLc);
    capR.retrieve(imgRc);
    cvtColor(imgLc, imgL_, COLOR_BGR2GRAY);
    cvtColor(imgRc, imgR_, COLOR_BGR2GRAY);

    Size imgL_size = imgL_.size();
    Size imgR_size = imgR_.size();
    if (imgL_size != imgR_size) {
        std::cout << "Camera inputs are not of the same dimension" << std::endl;
        exit(-1);
    }
    Size img_size = imgL_size;

    int middle = std::floor(img_size.height / 2);
    int height_delta = std::floor(img_size.height * (middle_prop / 2));
    i_min = middle - height_delta;
    i_max = middle + height_delta;

//    int color_mode = alg == STEREO_BM ? 0 : -1;

    FileStorage fs(intrinsic_filename, FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", intrinsic_filename.c_str());
        return -1;
    }
    if (scale != 1.f) {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(imgL_, temp1, Size(), scale, scale, method);
        imgL_ = temp1;
        resize(imgR_, temp2, Size(), scale, scale, method);
        imgR_ = temp2;
    }


    Rect roi1, roi2;
    Mat Q;

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scale;
    M2 *= scale;

    fs.open(extrinsic_filename, FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open file %s\n", extrinsic_filename.c_str());
        return -1;
    }

    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1,
                  &roi2);

    Ptr<StereoBM> bm = StereoBM::create(16, 9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 9);
    if (alg == STEREO_BM) {
        bm->setROI1(roi1);
        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
        bm->setMinDisparity(0);
        bm->setNumDisparities(numberOfDisparities);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(1);
    } else {
        int cn = imgL_.channels();

        sgbm->setPreFilterCap(63);
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        sgbm->setBlockSize(sgbmWinSize);
        sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
        sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
        sgbm->setMinDisparity(4);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(45);
        sgbm->setSpeckleRange(16);
//        sgbm->setDisp12MaxDiff(1);
        if (alg == STEREO_HH)
            sgbm->setMode(StereoSGBM::MODE_HH);
        else if (alg == STEREO_SGBM)
            sgbm->setMode(StereoSGBM::MODE_SGBM);
        else if (alg == STEREO_3WAY)
            sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    }


    Ptr<ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(sgbm);
    Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(sgbm);
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);

    Mat mapL1, mapL2, mapR1, mapR2;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, mapL1, mapL2);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, mapR1, mapR2);

    Point3d O(0, 0, 0);
    Point3d ihat(1, 0, 0);
    Point3d jhat(0, 1, 0);
    Point3d khat(0, 0, 1);
    viz::WLine linei(O, ihat, viz::Color::white());
    viz::WLine linej(O, jhat, viz::Color::red());
    viz::WLine linek(O, khat, viz::Color::blue());
    viz::Viz3d myWindow("Coordinate Frame");

    Mat xyz;
    Mat floatDisp;
    Mat disp, disp8, right_disp, disp_filtered;
    int64 t;
    std::vector<Point3f> buffer;
    std::vector<uint8_t> color;
    float disparity_multiplier = 1.0f;

    float euc_dist;
    float min_euc_dist = z_limit;
    Point3f xyz_point;
    Point3f min_dist_pt;
    while (true) {
        t = getTickCount();

        capL.grab();
        capR.grab();
        capL.retrieve(imgLc);
        capR.retrieve(imgRc);
        cvtColor(imgLc, imgL_, COLOR_BGR2GRAY);
        cvtColor(imgRc, imgR_, COLOR_BGR2GRAY);
        // reduce noise
//        fastNlMeansDenoising(imgL_, imgL_);
//        fastNlMeansDenoising(imgR_, imgR_);

        remap(imgL_, imgL, mapL1, mapL2, INTER_LINEAR);
        remap(imgR_, imgR, mapR1, mapR2, INTER_LINEAR);

        if (alg == STEREO_BM) {
            bm->compute(imgL, imgR, disp);
//            right_matcher->compute(imgR, imgL, right_disp);
//            wls_filter->filter(disp, imgLc, disp_filtered, right_disp);
            if (disp.type() == CV_16S) {
                disparity_multiplier = 16.0f;
            }
        } else if (alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY) {
            sgbm->compute(imgL, imgR, disp);

            if (disp.type() == CV_16S)
                disparity_multiplier = 16.0f;
        }

        namedWindow("left", 1);
        namedWindow("right", 1);
        namedWindow("disparity", 0);
        imshow("left", imgL_);
        imshow("right", imgR_);
        disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
        imshow("disparity", disp8);

//        if (alg != STEREO_VAR)
        disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
        reprojectImageTo3D(floatDisp, xyz, Q, true);

        buffer.clear();
        color.clear();
        for (int i = i_min; i < i_max; i++) {
            for (int j = 0; j < xyz.cols; j++) {
                xyz_point = xyz.at<Point3f>(i, j);
                if (xyz_point.z < z_limit && xyz_point.y >= y_range.x && xyz_point.y <= y_range.y) {
                    buffer.emplace_back(xyz_point.x, 0, xyz_point.z);
                    color.push_back(imgL.at<uint8_t >(i, j));  // this is for visualization only
                }
            }
        }

        t = getTickCount() - t;
        printf("Refresh Rate: %f Hz\n", 1 / (t / getTickFrequency()));

        // Everything beyond this point is for visualization
        if (buffer.empty()) {
            continue;
        }
        viz::WCloud cloud_widget = viz::WCloud(buffer, color);
        cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 5);
        if (!myWindow.wasStopped()) {
            myWindow.showWidget("Depth", cloud_widget);
            myWindow.showWidget("i", linei);
            myWindow.showWidget("j", linej);
            myWindow.showWidget("k", linek);
            myWindow.spinOnce(30, true);
        } else {
            break;
        }

        if (waitKey(27) == 30)
            break;
    }
    return 0;
}
