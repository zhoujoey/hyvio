//
// Created by xiaochen at 19-8-21.
// Managing the image processer and the estimator.
//

#ifndef SYSTEM_H
#define SYSTEM_H


#include <common_lib.h>


namespace larvio {

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
    bool initialize();

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
    ros::Publisher odom_pub;
    ros::Publisher stable_feature_pub;
    ros::Publisher active_feature_pub;
    ros::Publisher path_pub;
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
#endif

    // Msgs to be published.
    std::vector<std_msgs::Header> header_buffer;    // buffer for heads of msgs to be published

    // Msgs to be published.
    odom_ros odom_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr stable_feature_msg_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr active_feature_msg_ptr;
    path_ros path_msg;

    // Frame id
    std::string fixed_frame_id;
    std::string child_frame_id;

    // Pointer for image processer.
    ImageProcessorPtr ImgProcesser;

    // Pointer for estimator.
    LarVioPtr Estimator;

    // Directory for config file.
    std::string config_file;

    // Imu and image msg synchronized threshold.
    double imu_img_timeTh;

    // IMU message buffer.
    std::vector<ImuData> imu_msg_buffer;

    // Img message buffer.
    std::vector<ImageDataPtr> img_msg_buffer;

    /*
        * @brief loadParameters
        *    Load parameters from the parameter server.
        */
    bool loadParameters();

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

} // end namespace larvio


#endif  //SYSTEM_H
