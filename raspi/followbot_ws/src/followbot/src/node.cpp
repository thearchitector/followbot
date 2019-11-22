#include <cloud.h>
#include <node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seeker");

    PointCloud pc{};
    ros::NodeHandle n;
    image_transport::ImageTransport it{n};

    image_transport::Publisher point_cloud = it.advertise("point_cloud", 1);
    ros::Publisher human_pose = n.advertise<followbot::Point2>("human_pose", 1);
    ros::Rate loop_rate(1);

    pc.setupStereoCameras();

    while (ros::ok())
    {
        cv::Mat img = pc.collectPointCloud();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, img).toImageMsg();
        point_cloud.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}