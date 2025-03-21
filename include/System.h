//
// Created by xiaochen at 19-8-21.
// Managing the image processer and the estimator.
//

#ifndef SYSTEM_H
#define SYSTEM_H


#include <common_lib.h>
#include "parameters.hpp"

namespace hyvio {

/*
 * @brief Manager of the system.
 */
class System {
public:
    // Constructor
    System(node_ros& n);
    // Disable copy and assign constructors.
    System(const ImageProcessor&) = delete;
    System operator=(const System&) = delete;

    // Destructor.
    ~System();

    // Initialize the object.
    bool initialize(std::string &_config_file);

    typedef boost::shared_ptr<System> Ptr;
    typedef boost::shared_ptr<const System> ConstPtr;

private:

    // Ros node handle.
    node_ros nh;

#ifdef ROS1
    // Subscribers.
    ros::Subscriber img_sub;
    ros::Subscriber imu_sub;

    // Publishers.
    image_transport::Publisher vis_img_pub;
    std::shared_ptr<ros::Publisher> odom_pub;
    std::shared_ptr<ros::Publisher> stable_feature_pub;
    std::shared_ptr<ros::Publisher> active_feature_pub;
    std::shared_ptr<ros::Publisher> path_pub;
    // Msgs to be published.
    std::vector<header_ros> header_buffer;    // buffer for heads of msgs to be published

    inline double Stamp2Sec(const std_msgs::Header & header){
        return header.stamp.sec + header.stamp.nsec/1e9;
    };


    inline void Sec2Stamp(std_msgs::Header & header, const double time) {
        header.stamp = ros::Time().fromSec(time);
        return;
    }

    inline void SetTimeNow(std_msgs::Header & header){
        header.stamp = ros::Time::now();
    }

#else
    // Subscribers.
    rclcpp::Subscription<image_ros>::SharedPtr img_sub;
    rclcpp::Subscription<imu_ros>::SharedPtr imu_sub;

    // Publishers.
    image_transport::Publisher vis_img_pub;
    rclcpp::Publisher<odom_ros>::SharedPtr odom_pub;
    rclcpp::Publisher<pcl_ros>::SharedPtr stable_feature_pub;
    rclcpp::Publisher<pcl_ros>::SharedPtr active_feature_pub;
    rclcpp::Publisher<path_ros>::SharedPtr path_pub;
    // Msgs to be published.
    std::vector<header_ros> header_buffer;    // buffer for heads of msgs to be published

    inline double Stamp2Sec(const std_msgs::msg::Header & header){
        auto time = header.stamp;
        return rclcpp::Time(time).seconds();
    };

    inline void Sec2Stamp(std_msgs::msg::Header & header, const double time) {
        int32_t sec = std::floor(time);
        auto nanosec_d = (time - std::floor(time)) * 1e9;
        uint32_t nanosec = nanosec_d;
        header.stamp = rclcpp::Time(sec, nanosec);
        return; 
    }

    inline void SetTimeNow(std_msgs::msg::Header & header){
        header.stamp = rclcpp::Clock().now();
    }

#endif

    std::string imu_topic = "/imu0";
    std::string vis_img_topic = "visualization_image";
    std::string img_topic = "/cam0/image_raw";
    std::string odom_topic = "odom";
    std::string path_topic = "path";
    std::string stable_feature_topic = "stable_feature_point_cloud";
    std::string active_feature_topic = "active_feature_point_cloud";
    std::string fixed_frame_id = "world";
    std::string child_frame_id = "odom";
    double imu_rate = 200.0;
    // Imu and image msg synchronized threshold.
    double imu_img_timeTh;

    // Msgs to be published.
    odom_ros odom_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr stable_feature_msg_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr active_feature_msg_ptr;
    // pcl_ros stable_cloud_msg; 
    // pcl_ros active_cloud_msg;
    path_ros path_msg;



    // Pointer for image processer.
    ImageProcessorPtr ImgProcesser;

    // Pointer for estimator.
    HyVioPtr Estimator;

    // Directory for config file.
    std::string config_file;



    // IMU message buffer.
    std::vector<ImuData> imu_msg_buffer;

    // Img message buffer.
    std::vector<ImageDataPtr> img_msg_buffer;


    /*
        * @brief createRosIO
        *    Create ros publisher and subscirbers.
        */
    bool createRosIO();

    /*
        * @brief imageCallback
        *    Callback function for the monocular images.
        * @param image msg.
        */
    void imageCallback(const image_ros_ptr& msg);

    /*
        * @brief imuCallback
        *    Callback function for the imu message.
        * @param msg IMU msg.
        */
    void imuCallback(const imu_ros_ptr& msg);

    /*
        * @brief publishVIO
        *    Publish the results of VIO.
        * @param time The time stamp of output msgs.
        */
    void publishVIO(const time_ros& time);
};

typedef System::Ptr SystemPtr;
typedef System::ConstPtr SystemConstPtr;

} // end namespace hyvio


#endif  //SYSTEM_H
