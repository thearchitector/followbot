#include <cloud.h>
#include <node.h>
#include <human.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seeker");

    ros::NodeHandle n;
    image_transport::ImageTransport it{n};
    PointCloud pc{};
    HumanDetector hd{};

    image_transport::Publisher point_cloud = it.advertise("point_cloud", 1);
    ros::Publisher human_pose = n.advertise<followbot::Point2>("human_pose", 1);
    ros::Rate loop_rate(1);

    pc.setupStereoCameras();
    hd.setupNetwork();

    while (ros::ok())
    {
        cv::Mat rectifiedImg;
        cv::Mat xyz = pc.collectPointCloud(rectifiedImg);
        followbot::Point2 pose_msg = hd.getHumanPosition(rectifiedImg, xyz);

        sensor_msgs::ImagePtr pc_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, xyz).toImageMsg();
        point_cloud.publish(pc_msg);
        human_pose.publish(pose_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}