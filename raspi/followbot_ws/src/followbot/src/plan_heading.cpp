#include <plan_heading.hpp>

int main(int argc, char**argv) {
    ros::init(argc, argv, "plan_heading");

    #ifndef PRODUCTION_MSG_DISPLAYED
    #define PRODUCTION_MSG_DISPLAYED
    #ifdef PRODUCTION
    std::cout << "-- IN PRODUCTION MODE (will suppress data visualizations) --" << std::endl;
    #else
    std::cout << "-- NOT IN PRODUCTION MODE (will show data visualizations) --" << std::endl;
    #endif
    #endif

    std_msgs::UInt16 heading_msg;
    ros::NodeHandle n;
    ros::Subscriber buffer_subscriber = n.subscribe<followbot::Buffer>("point_cloud", 1, buffer_callback);
    ros::Subscriber human_pose_subscriber = n.subscribe<followbot::Buffer>("human_pose", 1, human_pose_callback);
    ros::Publisher desired_heading = n.advertise<std_msgs::UInt16>("desired_heading", 1);
    ros::Rate loop_rate(5);

    followbot::Buffer buffer_msg;
    followbot::Point2 pose_msg;

    while (ros::ok()) {
        #ifndef PRODUCTION
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        #endif


        heading_msg.data = (short)std::round(std::atan2(pose_msg.z, pose_msg.x));
        std::cout << "Person: " << pose_msg.x << ", " << pose_msg.z << std::endl;

        desired_heading.publish(heading_msg);
        ros::spinOnce();

        #ifdef PRODUCTION
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "plan_heading loop time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
        #endif

        loop_rate.sleep();
    }
    return 0;
}

void human_pose_callback(const followbot::Buffer &buffer_msg) {

}

void buffer_callback(const followbot::Buffer &buffer_msg) {

}