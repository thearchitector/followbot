#include <loc_and_pc.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "seeker");

    #ifdef PRODUCTION
    std::cout << "-- IN PRODUCTION MODE (will suppress data visualizations) -- " << std::endl;
    #else
    std::cout << "-- NOT IN PRODUCTION MODE (will show data visualizations) --" << std::endl;
    #endif

    ros::NodeHandle n;
    PointCloud pc{};
    HumanDetector hd{};

    ros::Publisher point_cloud = n.advertise<followbot::Buffer>("point_cloud", 1);
    ros::Publisher human_pose = n.advertise<followbot::Point2>("human_pose", 1);
    ros::Publisher desired_heading = n.advertise<std_msgs::UInt16>("desired_heading", 1);
    ros::Rate loop_rate(5);

    pc.setupStereoCameras();
    hd.setupNetwork();

    followbot::Buffer buffer_msg;
    followbot::Point2 pose_msg;
    std_msgs::UInt16 heading_msg;

    cv::Mat rectifiedImg;
    cv::Mat xyz;

    while (ros::ok()) {
        #ifndef PRODUCTION
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        #endif

        pc.collectPointCloud(rectifiedImg, xyz);
        hd.getHumanPosition(rectifiedImg, xyz, pose_msg);

        buffer_msg.buffer.clear();
        heading_msg.data = (short)std::round(std::atan2(pose_msg.z, pose_msg.x));
        std::cout << "Person: " << pose_msg.x << ", " << pose_msg.z << std::endl;

        #ifndef PRODUCTION
        pc.showPersonLoc(pose_msg);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Hz: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
        #endif

        point_cloud.publish(buffer_msg);
        human_pose.publish(pose_msg);
        desired_heading.publish(heading_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}