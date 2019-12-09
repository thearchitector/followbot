#include <seeker.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "seeker");
    cv::setUseOptimized(true);

    #ifdef PRODUCTION
    std::cout << "-- IN PRODUCTION MODE (will suppress data visualizations) --" << std::endl;
    #else
    std::cout << "-- NOT IN PRODUCTION MODE (will show data visualizations) --" << std::endl;
    #endif

    ros::NodeHandle n;
    PointCloud pc{};
    HumanDetector hd{};

    ros::Publisher world_publisher = n.advertise<followbot::World>("world", 1);
    ros::Rate loop_rate(5);

    pc.setupStereoCameras();
    hd.setupNetwork();

    followbot::World world_msg;

    cv::Mat rectifiedImg;
    cv::Mat xyz;

    while (ros::ok()) {
        #ifdef PRODUCTION
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        #endif

        pc.collectPointCloud(rectifiedImg, xyz, world_msg);
        hd.getHumanPosition(rectifiedImg, xyz, world_msg);

        #ifdef PRODUCTION
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Seeker Hz: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
        #else
//        pc.showPointCloud();
        #endif

        world_publisher.publish(world_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    pc.releaseCameras();
    return 0;
}