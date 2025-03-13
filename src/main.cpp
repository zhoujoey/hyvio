//
// Created by xiaochen at 19-8-21.
// Main function for LARVIO.
//

#include <ros/ros.h>
#include <System.h>

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "larvio");
    ros::NodeHandle nh("~");

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

