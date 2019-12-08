#include <plan_heading.hpp>

int main(int argc, char**argv) {
    ros::init(argc, argv, "plan_heading");

    AStar planner{};

    ros::NodeHandle n;
    ros::Subscriber world_subscriber = n.subscribe<followbot::World>("world", 1, &AStar::planHeading, &planner);
    ros::Publisher desired_heading = n.advertise<std_msgs::UInt16>("desired_heading", 1);
    ros::Rate loop_rate(5);

    followbot::World world_msg;
    std_msgs::UInt16 heading_msg;

    while (ros::ok()) {
        #ifndef PRODUCTION
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        #endif

//        heading_msg.data = planner.current_heading;
        heading_msg.data = (short)std::round(std::atan2(world_msg.person.x, world_msg.person.z));
        std::cout << "Person: " << world_msg.person.x << ", " << world_msg.person.z << std::endl;

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