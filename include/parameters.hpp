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
    int img_rate;

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
    double imu_rate;
    
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
    bool if_FEJ;
    int least_observation_number;
    bool if_ZUPT_valid;
    double zupt_max_feature_dis;
    double static_duration;
    int feature_idp_dim;
    bool use_schmidt;

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
            fast_threshold = yaml["fast_threshold"].as<int>();
            patch_size = yaml["patch_size"].as<int>();
            pyramid_levels = yaml["pyramid_levels"].as<int>();
            max_iteration = yaml["max_iteration"].as<int>();
            track_precision = yaml["track_precision"].as<double>();
            ransac_threshold = yaml["ransac_threshold"].as<double>();

            max_features_num = yaml["max_features_num"].as<int>();
            min_distance = yaml["min_distance"].as<int>();
            flag_equalize = yaml["flag_equalize"].as<bool>();

            pub_frequency = yaml["pub_frequency"].as<int>();
            img_rate = yaml["img_rate"].as<int>();

            // Output files directory
            output_dir = yaml["output_dir"].as<std::string>();

            /*
            * Camera calibration parameters
            */
            // Distortion model
            distortion_model = yaml["distortion_model"].as<std::string>();
            camera_model = yaml["camera_model"].as<std::string>();
            // Resolution of camera
            cam_resolution[0] = yaml["resolution_width"].as<int>();
            cam_resolution[1] = yaml["resolution_height"].as<int>();
            // Camera calibration instrinsics
            cam_intrinsics[0] = yaml["intrinsics"]["fx"].as<double>();
            cam_intrinsics[1] = yaml["intrinsics"]["fy"].as<double>();
            cam_intrinsics[2] = yaml["intrinsics"]["cx"].as<double>();
            cam_intrinsics[3] = yaml["intrinsics"]["cy"].as<double>();
            // Distortion coefficient
            cam_distortion_coeffs[0] = yaml["distortion_coeffs"]["k1"].as<double>();
            cam_distortion_coeffs[1] = yaml["distortion_coeffs"]["k2"].as<double>();
            cam_distortion_coeffs[2] = yaml["distortion_coeffs"]["p1"].as<double>();
            cam_distortion_coeffs[3] = yaml["distortion_coeffs"]["p2"].as<double>();
            //Extrinsics between camera and IMU
            const std::vector<double> T_data = yaml["T_cam_imu"]["data"].as<std::vector<double>>();
            R_cam_imu << T_data[0], T_data[1], T_data[2],
                         T_data[4], T_data[5], T_data[6],
                         T_data[8], T_data[9], T_data[10];
            t_cam_imu << T_data[3], T_data[7], T_data[11];

            imu_rate = yaml["imu_rate"].as<double>();
            position_std_threshold = yaml["position_std_threshold"].as<double>();
            rotation_threshold = yaml["rotation_threshold"].as<double>();
            translation_threshold = yaml["translation_threshold"].as<double>();
            tracking_rate_threshold = yaml["tracking_rate_threshold"].as<double>();
            max_track_len = yaml["max_track_len"].as<int>();
            feature_translation_threshold = yaml["feature_translation_threshold"].as<double>();
            reset_fej_threshold = yaml["reset_fej_threshold"].as<double>();
            td = yaml["td"].as<double>();
            estimate_td = yaml["estimate_td"].as<bool>();
            estimate_extrin = yaml["estimate_extrin"].as<bool>();
            noise_gyro = yaml["noise_gyro"].as<double>();
            noise_acc = yaml["noise_acc"].as<double>();
            noise_gyro_bias = yaml["noise_gyro_bias"].as<double>();
            noise_acc_bias = yaml["noise_acc_bias"].as<double>();
            noise_feature = yaml["noise_feature"].as<double>();
            zupt_noise_v = yaml["zupt_noise_v"].as<double>();
            zupt_noise_p = yaml["zupt_noise_p"].as<double>();
            zupt_noise_q = yaml["zupt_noise_q"].as<double>();
            initial_covariance_orientation = yaml["initial_covariance_orientation"].as<double>();
            initial_covariance_position = yaml["initial_covariance_position"].as<double>();
            initial_covariance_velocity = yaml["initial_covariance_velocity"].as<double>();
            initial_covariance_gyro_bias = yaml["initial_covariance_gyro_bias"].as<double>();
            initial_covariance_acc_bias = yaml["initial_covariance_acc_bias"].as<double>();
            initial_covariance_extrin_rot = yaml["initial_covariance_extrin_rot"].as<double>();
            initial_covariance_extrin_trans = yaml["initial_covariance_extrin_trans"].as<double>();
            calib_imu_instrinsic = yaml["calib_imu_instrinsic"].as<bool>();
            sw_size = yaml["sw_size"].as<int>();
            if_FEJ = yaml["if_FEJ"].as<bool>();
            least_observation_number = yaml["least_observation_number"].as<int>();
            if_ZUPT_valid = yaml["if_ZUPT_valid"].as<bool>();
            zupt_max_feature_dis = yaml["zupt_max_feature_dis"].as<double>();
            static_duration = yaml["static_duration"].as<double>();
            aug_grid_rows = yaml["aug_grid_rows"].as<int>();
            aug_grid_cols = yaml["aug_grid_cols"].as<int>();
            max_features_in_one_grid = yaml["max_features_in_one_grid"].as<int>();
            feature_idp_dim = yaml["feature_idp_dim"].as<int>();
            use_schmidt = yaml["use_schmidt"].as<bool>();

        } catch (...) {
            LOG(ERROR) << "Bad conversion in loading yaml config";
            return false;
        }
        return true;
    }

};

#endif