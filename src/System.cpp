//
// Created by xiaochen at 19-8-21.
// Managing the image processer and the estimator.
//



#include <System.h>

#include <iostream>



using namespace std;
using namespace cv;
using namespace Eigen;

namespace larvio {

System::System(node_ros& n) : nh(n) {}


System::~System() {
    // Clear buffer
    imu_msg_buffer.clear();
    img_msg_buffer.clear();
}


#ifdef ROS1
bool System::loadParameters() {
    // Configuration file path.
    nh.getParam("config_file", config_file);

    // Imu and img synchronized threshold.
    double imu_rate;
    nh.param<double>("imu_rate", imu_rate, 200.0);
    imu_img_timeTh = 1/(2*imu_rate);

    return true;
}

bool System::createRosIO() {
    // Subscribe imu msg.
    imu_sub = nh.subscribe("imu", 5000, &System::imuCallback, this);

    // Subscribe image msg.
    img_sub = nh.subscribe("cam0_image", 50, &System::imageCallback, this);

    // Advertise processed image msg.
    image_transport::ImageTransport it(nh);
    vis_img_pub = it.advertise("visualization_image", 1);

    // Advertise odometry msg.
    odom_pub = std::make_shared<ros::Publisher>(nh.advertise<odom_ros>("odom", 10));

    // Advertise point cloud msg.
    stable_feature_pub = std::make_shared<ros::Publisher>(nh.advertise<pcl_ros>("/stable_feature_point_cloud", 1));
    active_feature_pub = std::make_shared<ros::Publisher>(nh.advertise<pcl_ros>("/active_feature_point_cloud", 1));

    // Advertise path msg.
    path_pub = std::make_shared<ros::Publisher>(nh.advertise<path_ros>("path", 10));

    nh.param<string>("fixed_frame_id", fixed_frame_id, "world");
    nh.param<string>("child_frame_id", child_frame_id, "odom");
    stable_feature_msg_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    stable_feature_msg_ptr->header.frame_id = fixed_frame_id;
    stable_feature_msg_ptr->height = 1;
    return true;
}
#else
// Load parameters from launch file
bool System::loadParameters() {
    // Configuration file path.
    nh->declare_parameter("config_file", "");
    nh->get_parameter("config_file", config_file);

    // Imu and img synchronized threshold.
    double imu_rate = 200.0;
    nh->declare_parameter("imu_rate", 200.0);
    nh->get_parameter("imu_rate", imu_rate);
    imu_img_timeTh = 1/(2*imu_rate);

    return true;
}

bool System::createRosIO() {
    // Subscribe imu msg.
    auto sensor_qos = rclcpp::QoS(200000);
    imu_sub = nh->create_subscription<imu_ros>("/imu0", sensor_qos,
            [this](const imu_ros_ptr msg) { imuCallback(msg); } 
    );

    // Subscribe image msg.
    img_sub = nh->create_subscription<image_ros>("/cam0/image_raw", sensor_qos,
            [this](const image_ros_ptr msg) { imageCallback(msg); } 
    );

    // Advertise processed image msg.
    image_transport::ImageTransport it(nh);
    vis_img_pub = it.advertise("visualization_image", 1);

    // Advertise odometry msg.
    odom_pub = nh->create_publisher<odom_ros>("odom", 10);

    // Advertise point cloud msg.
    stable_feature_pub = nh->create_publisher<pcl_ros>("stable_feature_point_cloud", 1);
    active_feature_pub = nh->create_publisher<pcl_ros>("active_feature_point_cloud", 1);

    // Advertise path msg.
    path_pub = nh->create_publisher<path_ros>("path", 10);
    nh->declare_parameter("fixed_frame_id", "world");
    nh->get_parameter("fixed_frame_id", fixed_frame_id);
    nh->declare_parameter("child_frame_id", "robot");
    nh->get_parameter("child_frame_id", child_frame_id);
    stable_feature_msg_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    stable_feature_msg_ptr->header.frame_id = fixed_frame_id;
    stable_feature_msg_ptr->height = 1;

    return true;
}
#endif


// Initializing the system.
bool System::initialize() {
    // Load necessary parameters
    if (!loadParameters()) {
        return false;
    }
    LOG(INFO) << "System: Finish loading ROS parameters...";

    // Set pointers of image processer and estimator.
    ImgProcesser.reset(new ImageProcessor(config_file));
    Estimator.reset(new LarVio(config_file));

    // Initialize image processer and estimator.
    if (!ImgProcesser->initialize()) {
        LOG(WARNING) << "Image Processer initialization failed!";
        return false;
    }
    if (!Estimator->initialize()) {
        LOG(WARNING) << "Estimator initialization failed!";
        return false;
    }

    // Try subscribing msgs
    if (!createRosIO()) {
        return false;
    }
    LOG(INFO) << "System Manager: Finish creating ROS IO...";

    return true;
}


// Push imu msg into the buffer.
void System::imuCallback(const imu_ros_ptr& msg) {
    imu_msg_buffer.push_back(ImuData(Stamp2Sec(msg->header),
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
}


// Process the image and trigger the estimator.
void System::imageCallback(const image_ros_ptr& msg) {
    // Do nothing if no imu msg is received.
    if (imu_msg_buffer.empty()) {
        return;
    }
    cv_bridge::CvImageConstPtr cvCPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    larvio::ImageDataPtr msgPtr(new ImgData);
    msgPtr->timeStampToSec = Stamp2Sec(cvCPtr->header);
    msgPtr->image = cvCPtr->image.clone();
    header_ros header = cvCPtr->header;

    // Decide if use img msg in buffer.
    bool bUseBuff = false;
    if (!imu_msg_buffer.empty() ||
        (imu_msg_buffer.end()-1)->timeStampToSec-msgPtr->timeStampToSec<-imu_img_timeTh) {
        img_msg_buffer.push_back(msgPtr);
        header_buffer.push_back(header);
    }
    
    if (!img_msg_buffer.empty()) {
        if ((imu_msg_buffer.end()-1)->timeStampToSec-(*(img_msg_buffer.begin()))->timeStampToSec<-imu_img_timeTh) {
            return;
        }
        bUseBuff = true;
    }

    if (!bUseBuff) {
        MonoCameraMeasurementPtr features = new MonoCameraMeasurement;

        // Process image to get feature measurement.
        bool bProcess = ImgProcesser->processImage(msgPtr, imu_msg_buffer, features);

        // Filtering if get processed feature.
        bool bPubOdo = false;
        if (bProcess) {
            bPubOdo = Estimator->processFeatures(features, imu_msg_buffer);
        }

        // Publish msgs if necessary
        if (bProcess) {
            cv_bridge::CvImage _image(header, "bgr8", ImgProcesser->getVisualImg());
            vis_img_pub.publish(_image.toImageMsg());
        }
        if (bPubOdo) {
            publishVIO(header.stamp);
        }

        delete features;
        return;
    } else {
        // Loop for using all the img in the buffer that satisfy the condition.
        int counter = 0;
        for (int i = 0; i < img_msg_buffer.size(); ++i) {
            // Break the loop if imu data is not enough
            if ((imu_msg_buffer.end()-1)->timeStampToSec-img_msg_buffer[i]->timeStampToSec<-imu_img_timeTh) {
                break;
            }

            MonoCameraMeasurementPtr features = new MonoCameraMeasurement;

            // Process image to get feature measurement.
            bool bProcess = ImgProcesser->processImage(img_msg_buffer[i], imu_msg_buffer, features);

            // Filtering if get processed feature.
            bool bPubOdo = false;
            if (bProcess) {
                bPubOdo = Estimator->processFeatures(features, imu_msg_buffer);
            }

            // Publish msgs if necessary
            if (bProcess) {
                cv_bridge::CvImage _image(header_buffer[i], "bgr8", ImgProcesser->getVisualImg());
                vis_img_pub.publish(_image.toImageMsg());
            }
            if (bPubOdo) {
                publishVIO(header_buffer[i].stamp);
            }

            delete features;
            counter++;
        }
        img_msg_buffer.erase(img_msg_buffer.begin(), img_msg_buffer.begin()+counter);
        header_buffer.erase(header_buffer.begin(), header_buffer.begin()+counter);
    }
}


// Publish informations of VIO, including odometry, path, points cloud and whatever needed.
void System::publishVIO(const time_ros& time) {
    // construct odometry msg
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = fixed_frame_id;
    odom_msg.child_frame_id = child_frame_id;
    Eigen::Isometry3d T_b_w = Estimator->getTbw();
    Eigen::Vector3d body_velocity = Estimator->getVel();
    Matrix<double, 6, 6> P_body_pose = Estimator->getPpose();
    Matrix3d P_body_vel = Estimator->getPvel();

    odom_msg.pose.pose.position.x = T_b_w.translation().x();
    odom_msg.pose.pose.position.y = T_b_w.translation().y();
    odom_msg.pose.pose.position.z = T_b_w.translation().z();
    
    // 提取旋转（四元数）
    Eigen::Quaterniond q(T_b_w.rotation());
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();

    odom_msg.twist.twist.linear.x = body_velocity.x();
    odom_msg.twist.twist.linear.y = body_velocity.y();
    odom_msg.twist.twist.linear.z = body_velocity.z();

    // construct path msg
    path_msg.header.stamp = time;
    path_msg.header.frame_id = fixed_frame_id;
    geometry_ros curr_path;
    curr_path.header.stamp = time;
    curr_path.header.frame_id = fixed_frame_id;
    curr_path.pose = odom_msg.pose.pose;
    path_msg.poses.push_back(curr_path);

    // construct point cloud msg
    // Publish the 3D positions of the features.
    // Including stable and active ones.
    // --Stable features
    std::map<larvio::FeatureIDType,Eigen::Vector3d> StableMapPoints;
    Estimator->getStableMapPointPositions(StableMapPoints);
    for (const auto& item : StableMapPoints) {
        const auto& feature_position = item.second;
        stable_feature_msg_ptr->points.push_back(pcl::PointXYZ(
                feature_position(0), feature_position(1), feature_position(2)));
    }
    stable_feature_msg_ptr->width = stable_feature_msg_ptr->points.size();
    // --Active features
    active_feature_msg_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    active_feature_msg_ptr->header.frame_id = fixed_frame_id;
    active_feature_msg_ptr->height = 1;
    std::map<larvio::FeatureIDType,Eigen::Vector3d> ActiveMapPoints;
    Estimator->getActiveeMapPointPositions(ActiveMapPoints);
    for (const auto& item : ActiveMapPoints) {
        const auto& feature_position = item.second;
        active_feature_msg_ptr->points.push_back(pcl::PointXYZ(
                feature_position(0), feature_position(1), feature_position(2)));
    }
    active_feature_msg_ptr->width = active_feature_msg_ptr->points.size();

    pcl_ros stable_cloud_msg;
    pcl::toROSMsg(*stable_feature_msg_ptr, stable_cloud_msg);
    stable_cloud_msg.header.stamp = time;
    stable_cloud_msg.header.frame_id = fixed_frame_id;
    
    pcl_ros active_cloud_msg;
    pcl::toROSMsg(*active_feature_msg_ptr, active_cloud_msg);
    active_cloud_msg.header.stamp = time;
    active_cloud_msg.header.frame_id = fixed_frame_id;

    stable_feature_pub->publish(stable_cloud_msg);
    active_feature_pub->publish(active_cloud_msg);

    odom_pub->publish(odom_msg);
    path_pub->publish(path_msg);
}

} // end namespace larvio
