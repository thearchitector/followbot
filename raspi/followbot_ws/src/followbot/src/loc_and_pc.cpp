#include <loc_and_pc.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "seeker");

    ros::NodeHandle n;
    PointCloud pc{};
    HumanDetector hd{};

    ros::Publisher human_pose = n.advertise<followbot::Point2>("human_pose", 1);
    ros::Rate loop_rate(30);

    pc.setupStereoCameras();
    hd.setupNetwork();

    while (ros::ok()) {
        cv::Mat rectifiedImg;
        followbot::Point2 pose_msg;
        cv::Mat xyz;

        pc.collectPointCloud(rectifiedImg, xyz);
        hd.getHumanPosition(rectifiedImg, xyz, pose_msg);

        std::cout << pose_msg.x << ", " << pose_msg.z << std::endl;

        if (hd.view) pc.showPersonLoc(pose_msg);

//        human_pose.publish(pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}