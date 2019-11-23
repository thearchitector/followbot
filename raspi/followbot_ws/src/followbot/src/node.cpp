#include <cloud.h>
#include <human.h>
#include <node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seeker");

    ros::NodeHandle n;
    PointCloud pc{};
    HumanDetector hd{};

    ros::Publisher map_obstacles = n.advertise<costmap_converter::ObstacleArrayMsg>("followbot/obstacles", 1);
    ros::Publisher human_pose = n.advertise<followbot::Point2>("human_pose", 1);
    ros::Rate loop_rate(1);

    pc.setupStereoCameras();
    hd.setupNetwork();

    while (ros::ok())
    {
        cv::Mat rectifiedImg;
        followbot::Point2 pose_msg;
        costmap_converter::ObstacleArrayMsg obstacle_msg;

        cv::Mat xyz = pc.collectPointCloud(rectifiedImg, obstacle_msg);
        hd.getHumanPosition(rectifiedImg, xyz, pose_msg);

        map_obstacles.publish(obstacle_msg);
        human_pose.publish(pose_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}