//
// Created by xiaochen at 19-8-21.
// Main function for HYVIO.
//


#include <System.h>
#include <common_lib.h>

#ifdef ROS1

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "vio_slam");
    node_ros nh("~");

    // Initialize google logging
    google::InitGoogleLogging(argv[0]);
    
    // Create HYVIO system
    hyvio::System system(nh);
    std::string config_file;
    nh.param<std::string>("config_file", config_file, "");
    if (config_file == "") {
        LOG(ERROR) << "can not load params, config_file is empty";
        return -1;
    } else {
        LOG(INFO) << "param path is : " << config_file;
    }
    // Initialize the system
    if (!system.initialize(config_file)) {
        ROS_ERROR("Failed to initialize System!");
        return -1;
    }
    
    ROS_INFO("HYVIO initialized successfully");

    // Spin and process callbacks
    ros::spin();

    // Clean up google logging
    google::ShutdownGoogleLogging();
    
    return 0;
}
#else
int main(int argc, char **argv) {
    // Initialize ROS node
    rclcpp::init(argc, argv);
    node_ros nh = std::make_shared<rclcpp::Node>("vio_slam");
    LOG(INFO) << "HYVIO system created1";
    // Initialize google logging
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;
    FLAGS_v = 1;

    google::InitGoogleLogging("lidar_slam");
    LOG(INFO) << "HYVIO system created2";
    // Create HYVIO system
    hyvio::System system(nh);
    LOG(INFO) << "HYVIO system created3";
    std::string config_file;
    nh->declare_parameter("config_file", "");
    nh->get_parameter("config_file", config_file);
    if (config_file == "") {
        LOG(ERROR) << "can not load params, config_file is empty";
        return -1;
    } else {
        LOG(INFO) << "param path is : " << config_file;
    }
    // Initialize the system
    if (!system.initialize(config_file)) {
        LOG(ERROR) << "Failed to initialize System!";
        return -1;
    }
    
    LOG(INFO) << "HYVIO initialized successfully";

    // Spin and process callbacks
    rclcpp::spin(nh);

    // Clean up google logging
    google::ShutdownGoogleLogging();
    
    return 0;
}
#endif
