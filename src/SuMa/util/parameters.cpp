#include "SuMa/util/parameters.h"

// Params for image
double INIT_DEPTH;
double MIN_PARALLAX;
int ROLLING_SHUTTER;
int ROW, COL;


// Params for laser scan
int DEPTH_ROW, DEPTH_COL;
float FOV_UP, FOV_DOWN;
float MAX_DEPTH, MIN_DEPTH;
float MAX_YAW, MIN_YAW;
bool FILTER_VERTEXMAP, USW_FILTERED_VERTEXMAP;
float BILATERAL_SPACE, BILATERAL_RANGE;
int SCAN_FREQUENCY;
std::string SCAN_TOPIC;

// Params for imu
double ACC_N, ACC_W;
double GYR_N, GYR_W;
double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
int IMU_FREQUENCY;
std::string IMU_TOPIC;
Eigen::Vector3d G{0.0, 0.0, 9.8};

// Params for calibration
std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
double TD;
double TR;
int ESTIMATE_TD;
int ESTIMATE_EXTRINSIC;
std::string EX_CALIB_RESULT_PATH;

// Params for optimization
double SOLVER_TIME;
int NUM_ITERATIONS;
std::string RESULT_PATH;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // Params for laser scan
    fsSettings["scan_topic"] >> SCAN_TOPIC;
    fsSettings["scan_frequency"] >> SCAN_FREQUENCY;
    fsSettings["depth_img_width"] >> DEPTH_COL;
    fsSettings["depth_img_height"] >> DEPTH_ROW;
    fsSettings["data_fov_up"] >> FOV_UP;
    fsSettings["data_fov_down"] >> FOV_DOWN;
    fsSettings["min_depth"] >> MIN_DEPTH;
    fsSettings["max_depth"] >> MAX_DEPTH;
    fsSettings["min_yaw"] >> MIN_YAW;
    fsSettings["max_yaw"] >> MAX_YAW;
    fsSettings["filter_vertexmap"] >> FILTER_VERTEXMAP;
    fsSettings["use_filtered_vertexmap"] >> USW_FILTERED_VERTEXMAP;
    fsSettings["bilateral_sigma_space"] >> BILATERAL_SPACE;
    fsSettings["bilateral_sigma_range"] >> BILATERAL_RANGE;
    
    // Params for imu
    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["imu_frequency"] >> IMU_FREQUENCY;
    

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    RESULT_PATH = OUTPUT_PATH + "/result.csv";
    std::cout << "result path " << RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
}
