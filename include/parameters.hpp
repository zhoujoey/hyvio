#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <glog/logging.h>
class Parameters
{
public:
    int fast_threshold;
    int patch_size;
    int pyramid_levels;
    int max_iteration;
    double track_precision;
    double ransac_threshold;
    int max_features_num;
    int min_distance;
    bool flag_equalize;
    int pub_frequency;
    int image_rate;

    // 输出配置
    std::string output_dir;

    // 相机内参
    std::string distortion_model;
    std::string camera_model;
    std::vector<int> cam_resolution;          // [width, height]
    std::vector<double> cam_intrinsics;       // [fx, fy, cx, cy]
    std::vector<double> cam_distortion_coeffs;// [k1, k2, p1, p2]
    
    // IMU-相机外参
    Eigen::Matrix3d R_cam_imu;          // 旋转矩阵
    Eigen::Vector3d t_cam_imu;  


    // Feature tracking parameters
    
    double position_std_threshold;
    double rotation_threshold;
    double translation_threshold;
    double tracking_rate_threshold;
    int max_track_len;
    double feature_translation_threshold;
    double reset_fej_threshold;
    double td;
    bool estimate_td;
    bool estimate_extrin;
    double noise_gyro;
    double noise_acc;
    double noise_gyro_bias;
    double noise_acc_bias;
    double noise_feature;
    double zupt_noise_v;
    double zupt_noise_p;
    double zupt_noise_q;
    double initial_covariance_orientation;
    double initial_covariance_position;
    double initial_covariance_velocity;
    double initial_covariance_gyro_bias;
    double initial_covariance_acc_bias;
    double initial_covariance_extrin_rot;
    double initial_covariance_extrin_trans;
    int aug_grid_rows;
    int aug_grid_cols;
    int max_features_in_one_grid;
    bool calib_imu_instrinsic;
    int sw_size;
    bool if_first_estimates_jacobian;
    int least_observation_number;
    bool if_ZUPT_valid;
    double zupt_max_feature_dis;
    double static_duration;
    int feature_idp_dim;
    bool use_schmidt;
    double imu_rate;
    std::string imu_topic;
    std::string vis_img_topic;
    std::string img_topic;
    std::string odom_topic;
    std::string path_topic;
    std::string stable_feature_topic;
    std::string active_feature_topic;
    std::string fixed_frame_id;
    std::string child_frame_id;

public:
    Parameters()
    {
        cam_resolution = std::vector<int>(2, 0);
        cam_intrinsics = std::vector<double>(4, 0.0);
        cam_distortion_coeffs = std::vector<double>(4, 0.0);
    }
    ~Parameters()
    {
        
    }



    bool LoadParamsFromYAML(const std::string &yaml_file) {
        LOG(INFO) << "Loading parameters from yaml file: " << yaml_file;
        // get params from yaml
        auto yaml = YAML::LoadFile(yaml_file);
        try {
            // Load common parameters
            output_dir = yaml["common"]["output_dir"].as<std::string>();
            imu_rate = yaml["common"]["imu_rate"].as<double>();
            image_rate = yaml["common"]["image_rate"].as<int>();
            imu_topic = yaml["common"]["imu_topic"].as<std::string>();
            vis_img_topic = yaml["common"]["vis_img_topic"].as<std::string>();
            img_topic = yaml["common"]["img_topic"].as<std::string>();
            odom_topic = yaml["common"]["odom_topic"].as<std::string>();
            path_topic = yaml["common"]["path_topic"].as<std::string>();
            stable_feature_topic = yaml["common"]["stable_feature_topic"].as<std::string>();
            active_feature_topic = yaml["common"]["active_feature_topic"].as<std::string>();
            fixed_frame_id = yaml["common"]["fixed_frame_id"].as<std::string>();
            child_frame_id = yaml["common"]["child_frame_id"].as<std::string>();

            if_first_estimates_jacobian = yaml["common"]["if_first_estimates_jacobian"].as<bool>();
            estimate_extrin = yaml["common"]["estimate_extrin"].as<bool>();
            estimate_td = yaml["common"]["estimate_td"].as<bool>();
            calib_imu_instrinsic = yaml["common"]["calib_imu_instrinsic"].as<bool>();

            // Camera parameters
            camera_model = yaml["camera"]["camera_model"].as<std::string>();
            distortion_model = yaml["camera"]["distortion_model"].as<std::string>();
            cam_resolution[0] = yaml["camera"]["resolution_width"].as<int>();
            cam_resolution[1] = yaml["camera"]["resolution_height"].as<int>();

            cam_intrinsics[0] = yaml["camera"]["intrinsics"]["fx"].as<double>();
            cam_intrinsics[1] = yaml["camera"]["intrinsics"]["fy"].as<double>();
            cam_intrinsics[2] = yaml["camera"]["intrinsics"]["cx"].as<double>();
            cam_intrinsics[3] = yaml["camera"]["intrinsics"]["cy"].as<double>();

            cam_distortion_coeffs[0] = yaml["camera"]["distortion_coeffs"]["k1"].as<double>();
            cam_distortion_coeffs[1] = yaml["camera"]["distortion_coeffs"]["k2"].as<double>();
            cam_distortion_coeffs[2] = yaml["camera"]["distortion_coeffs"]["p1"].as<double>();
            cam_distortion_coeffs[3] = yaml["camera"]["distortion_coeffs"]["p2"].as<double>();

            const std::vector<double> T_data = yaml["camera"]["T_cam_imu"]["data"].as<std::vector<double>>();
            R_cam_imu << T_data[0], T_data[1], T_data[2],
                         T_data[4], T_data[5], T_data[6],
                         T_data[8], T_data[9], T_data[10];
            t_cam_imu << T_data[3], T_data[7], T_data[11];
            td = yaml["camera"]["td"].as<double>();

            // Front-end parameters
            pyramid_levels = yaml["front_end"]["pyramid_levels"].as<int>();
            patch_size = yaml["front_end"]["patch_size"].as<int>();
            fast_threshold = yaml["front_end"]["fast_threshold"].as<int>();
            max_iteration = yaml["front_end"]["max_iteration"].as<int>();
            track_precision = yaml["front_end"]["track_precision"].as<double>();
            ransac_threshold = yaml["front_end"]["ransac_threshold"].as<double>();
            max_features_num = yaml["front_end"]["max_features_num"].as<int>();
            min_distance = yaml["front_end"]["min_distance"].as<int>();
            flag_equalize = yaml["front_end"]["flag_equalize"].as<bool>();
            pub_frequency = yaml["front_end"]["pub_frequency"].as<int>();

            // Back-end parameters
            sw_size = yaml["back_end"]["sw_size"].as<int>();

            position_std_threshold = yaml["back_end"]["position_std_threshold"].as<double>();
            rotation_threshold = yaml["back_end"]["rotation_threshold"].as<double>();
            translation_threshold = yaml["back_end"]["translation_threshold"].as<double>();
            tracking_rate_threshold = yaml["back_end"]["tracking_rate_threshold"].as<double>();

            least_observation_number = yaml["back_end"]["least_observation_number"].as<int>();
            max_track_len = yaml["back_end"]["max_track_len"].as<int>();
            feature_translation_threshold = yaml["back_end"]["feature_translation_threshold"].as<double>();

            noise_gyro = yaml["back_end"]["noise_gyro"].as<double>();
            noise_acc = yaml["back_end"]["noise_acc"].as<double>();
            noise_gyro_bias = yaml["back_end"]["noise_gyro_bias"].as<double>();
            noise_acc_bias = yaml["back_end"]["noise_acc_bias"].as<double>();
            noise_feature = yaml["back_end"]["noise_feature"].as<double>();
            
            initial_covariance_orientation = yaml["back_end"]["initial_covariance_orientation"].as<double>();
            initial_covariance_velocity = yaml["back_end"]["initial_covariance_velocity"].as<double>();
            initial_covariance_position = yaml["back_end"]["initial_covariance_position"].as<double>();
            initial_covariance_gyro_bias = yaml["back_end"]["initial_covariance_gyro_bias"].as<double>();
            initial_covariance_acc_bias = yaml["back_end"]["initial_covariance_acc_bias"].as<double>();
            initial_covariance_extrin_rot = yaml["back_end"]["initial_covariance_extrin_rot"].as<double>();
            initial_covariance_extrin_trans = yaml["back_end"]["initial_covariance_extrin_trans"].as<double>();

            reset_fej_threshold = yaml["back_end"]["reset_fej_threshold"].as<double>();

            if_ZUPT_valid = yaml["back_end"]["if_ZUPT_valid"].as<bool>();
            zupt_max_feature_dis = yaml["back_end"]["zupt_max_feature_dis"].as<double>();
            zupt_noise_v = yaml["back_end"]["zupt_noise_v"].as<double>();
            zupt_noise_p = yaml["back_end"]["zupt_noise_p"].as<double>();
            zupt_noise_q = yaml["back_end"]["zupt_noise_q"].as<double>();
            
            static_duration = yaml["back_end"]["static_duration"].as<double>();

            max_features_in_one_grid = yaml["back_end"]["max_features_in_one_grid"].as<int>();
            aug_grid_rows = yaml["back_end"]["aug_grid_rows"].as<int>();
            aug_grid_cols = yaml["back_end"]["aug_grid_cols"].as<int>();
            feature_idp_dim = yaml["back_end"]["feature_idp_dim"].as<int>();

            use_schmidt = yaml["back_end"]["use_schmidt"].as<bool>();

        } catch (...) {
            LOG(ERROR) << "Bad conversion in loading yaml config";
            return false;
        }
        return true;
    }

};

#endif