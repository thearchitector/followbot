#include <loc_and_pc.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "seeker");

    #ifdef PRODUCTION
    std::cout << "-- IN PRODUCTION MODE -- (will suppress data visualizations) " << std::endl;
    #else
    std::cout << "-- NOT IN PRODUCTION MODE (will show data visualizations) --" << std::endl;
    #endif

    ros::NodeHandle n;
    PointCloud pc{};
    HumanDetector hd{};

    ros::Publisher human_pose = n.advertise<followbot::Point2>("human_pose", 1);
    ros::Rate loop_rate(30);

    pc.setupStereoCameras();
    hd.setupNetwork();

    cv::Mat rectifiedImg;
    followbot::Point2 pose_msg;
    cv::Mat xyz;
    while (ros::ok()) {
//        std::clock_t begin = clock();
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        pc.collectPointCloud(rectifiedImg, xyz);
        hd.getHumanPosition(rectifiedImg, xyz, pose_msg);

        std::cout << "Person loc: " << pose_msg.x << ", " << pose_msg.z << std::endl;

        #ifndef PRODUCTION
        pc.showPersonLoc(pose_msg);
        #endif

        human_pose.publish(pose_msg);
        ros::spinOnce();

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "loop time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<
            "ms" << std::endl;

        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}