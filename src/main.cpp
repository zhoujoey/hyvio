//
// Created by xiaochen at 19-8-21.
// Main function for LARVIO.
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
    
    // Create LARVIO system
    larvio::System system(nh);
    
    // Initialize the system
    if (!system.initialize()) {
        ROS_ERROR("Failed to initialize System!");
        return -1;
    }
    
    ROS_INFO("LARVIO initialized successfully");

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

    // Initialize google logging
    google::InitGoogleLogging(argv[0]);
    
    // Create LARVIO system
    larvio::System system(nh);
    
    // Initialize the system
    if (!system.initialize()) {
        LOG(ERROR) << "Failed to initialize System!";
        return -1;
    }
    
    LOG(INFO) << "LARVIO initialized successfully";

    // Spin and process callbacks
    rclcpp::spin(nh);

    // Clean up google logging
    google::ShutdownGoogleLogging();
    
    return 0;
}
#endif
