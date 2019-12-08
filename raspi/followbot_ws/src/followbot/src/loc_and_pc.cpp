#include <loc_and_pc.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "loc_and_pc");

    #ifndef PRODUCTION_MSG_DISPLAYED
    #define PRODUCTION_MSG_DISPLAYED
    #ifdef PRODUCTION
    std::cout << "-- IN PRODUCTION MODE (will suppress data visualizations) -- " << std::endl;
    #else
    std::cout << "-- NOT IN PRODUCTION MODE (will show data visualizations) --" << std::endl;
    #endif
    #endif

    ros::NodeHandle n;
    PointCloud pc{};
    HumanDetector hd{};

    ros::Publisher point_cloud = n.advertise<followbot::Buffer>("point_cloud", 1);
    ros::Publisher human_pose = n.advertise<followbot::Point2>("human_pose", 1);
    ros::Rate loop_rate(5);

    pc.setupStereoCameras();
    hd.setupNetwork();

    followbot::Buffer buffer_msg;
    followbot::Point2 pose_msg;

    cv::Mat rectifiedImg;
    cv::Mat xyz;

    while (ros::ok()) {
        #ifndef PRODUCTION
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        #endif

        pc.collectPointCloud(rectifiedImg, xyz, buffer_msg);
        hd.getHumanPosition(rectifiedImg, xyz, pose_msg);
        human_pose.publish(pose_msg);
        point_cloud.publish(buffer_msg);
        ros::spinOnce();

        #ifdef PRODUCTION
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "loc_and_pc loop time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
        #else
        pc.showPointCloud();
        #endif

        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}