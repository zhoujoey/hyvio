#ifndef COMMON_LIB_H
#define COMMON_LIB_H


#include <vector>
#include <boost/shared_ptr.hpp>

#include <larvio/image_processor.h>
#include <larvio/larvio.h>
#include <glog/logging.h>
#include "sensors/ImuData.hpp"
#include "sensors/ImageData.hpp"

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#ifdef ROS1
    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <message_filters/subscriber.h>
    #include <sensor_msgs/Imu.h>
    #include <sensor_msgs/Image.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <nav_msgs/Odometry.h>
    #include <nav_msgs/Path.h>


    #include <tf2_ros/buffer.h>
    #include <tf2_ros/transform_broadcaster.h>
    #include <tf2_ros/transform_listener.h>
    #include <visualization_msgs/Marker.h>


    using pcl_ros_ptr = sensor_msgs::PointCloud2::ConstPtr;
    using pcl_ros = sensor_msgs::PointCloud2;
    using image_ros = sensor_msgs::Image;
    using image_ros_ptr = sensor_msgs::Image::ConstPtr;
    using imu_ros_ptr = sensor_msgs::Imu::ConstPtr;
    using imu_ros = sensor_msgs::Imu;
    using path_ros = nav_msgs::Path;
    using odom_ros = nav_msgs::Odometry;
    using node_ros = ros::NodeHandle;

    using time_ros = ros::Time;


#else
    #include <rclcpp/rclcpp.hpp>
    #include <image_transport/image_transport.hpp>
    #include <message_filters/subscriber.hpp>
    #include <sensor_msgs/msg/imu.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <sensor_msgs/msg/point_cloud2.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <nav_msgs/msg/path.hpp>


    #include <tf2_ros/buffer.hpp>
    #include <tf2_ros/transform_broadcaster.hpp>
    #include <tf2_ros/transform_listener.hpp>
    #include <visualization_msgs/msg/marker.hpp>


    using pcl_ros = sensor_msgs::msg::PointCloud2;
    using pcl_ros_ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
    using image_ros = sensor_msgs::msg::Image;
    using image_ros_ptr = sensor_msgs::msg::Image::SharedPtr;
    using imu_ros = sensor_msgs::msg::Imu;
    using imu_ros_ptr = sensor_msgs::msg::Imu::SharedPtr;
    using path_ros = nav_msgs::msg::Path;
    using odom_ros = nav_msgs::msg::Odometry;
    using node_ros = rclcpp::Node::SharedPtr;
    using time_ros = rclcpp::Time;

#endif  


#endif  //COMMON_LIB_H
