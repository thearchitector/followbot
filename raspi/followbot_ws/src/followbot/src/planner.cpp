#include <planner.hpp>

int main(int argc, char**argv) {
    ros::init(argc, argv, "planner");

    AStar planner{};

    ros::NodeHandle n;
    ros::Subscriber world_subscriber = n.subscribe<followbot::World>("world", 1, &AStar::planHeading, &planner);
    ros::Publisher desired_heading = n.advertise<std_msgs::UInt16>("desired_heading", 1);
    ros::Rate loop_rate(5);

    std_msgs::UInt16 heading_msg;

    while (ros::ok()) {
        #ifdef PRODUCTION
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        #endif

        heading_msg.data = planner.current_heading;
        desired_heading.publish(heading_msg);

        #ifdef PRODUCTION
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Planner Hz: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
        #endif

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}