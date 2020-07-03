#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>

#include <SuMa/core/SurfelMapping.h>
#include "SuMa/rv/LaserscanReader.h"
#include "SuMa/opengl/Model.h"
#include "SuMa/util/kitti_utils.h"
#include "SuMa/util/ScanAccumulator.h"
#include "SuMa/util/parameters.h"

// SumaWorker suma_worker;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloud2ConstPtr> feature_buf;
queue<sensor_msgs::PointCloud2ConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

void laserScanHandler(const sensor_msgs::PointCloud2ConstPtr& laserScanIn);
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);

void process()
{
    while(true)
    {
	if (!filename.isNull()) {
    QString title = "LaserFusion";
    title.append(" - ");
    QStringList parts = filename.split("/");
    title.append(QString::fromStdString(FileUtil::dirName(filename.toStdString())));
    setWindowTitle(title);

    std::shared_ptr<Model> model;
    ui_.wCanvas->setRobotModel(model);

    std::string extension = "INVALID";

    QFileInfo file_info(filename);
    if (file_info.isDir()) {
      QDir directory(filename);
      QStringList files = directory.entryList(QDir::NoFilter, QDir::Name);

      std::map<std::string, int32_t> suffix_count;
      for (int32_t i = 0; i < files.size(); ++i) {
        QFileInfo info(directory.filePath(files[i]));
        if (!info.isFile()) continue;

        std::string ext = info.suffix().toStdString();
        if (suffix_count.find(ext) == suffix_count.end()) suffix_count[ext] = 0;
        suffix_count[ext] += 1;
      }

      if (suffix_count.size() > 0) {
        int32_t maxCount = suffix_count.begin()->second;
        extension = std::string(".") + suffix_count.begin()->first;

        for (auto& entry : suffix_count) {
          if (entry.second > maxCount) {
            extension = std::string(".") + entry.first;
            maxCount = entry.second;
          }
        }

        std::cout << "Heuristically found '" << extension << "' as extension." << std::endl;
      }
    } else if (file_info.isFile()) {
      extension = std::string(".") + file_info.suffix().toStdString();
    }

    if (extension == ".bin") {
      delete reader_;
      reader_ = new KITTIReader(filename.toStdString(), 50);

      scanFrequency = 10.0f;   // in Hz.
      timer_.setInterval(10);  // 1./10. second = 100 msecs.

      if (fusion_ != nullptr) fusion_->reset();
      ui_.wCanvas->reset();

      uint32_t N = reader_->count();
      precomputed_.resize(N);
      oldPoses_.resize(N);

      for (uint32_t i = 0; i < N; ++i) {
        precomputed_[i] = false;
      }
      if (oldPoses_.size() > 0) oldPoses_[0] = Eigen::Matrix4f::Identity();

      calib_.clear();
      groundtruth_.clear();

      if (parts.size() > 2) {
        std::string calibration_file;
        for (int32_t i = 0; i < parts.size() - 2; ++i) {
          calibration_file += parts[i].toStdString();
          calibration_file += "/";
        }
        calibration_file += "calib.txt";

        std::cout << "calibration filename: " << calibration_file << "..." << std::flush;

        if (rv::FileUtil::exists(calibration_file)) {
          calib_.initialize(calibration_file);
          std::cout << "loaded." << std::endl;

          ui_.wCanvas->setCalibration(calib_);
        } else {
          std::cout << "not found." << std::endl;
        }
      }

      if (parts.size() > 3 && calib_.exists("Tr")) {
        std::string sequence = parts[parts.size() - 3].toStdString();

        Eigen::Matrix4f Tr = calib_["Tr"];
        Eigen::Matrix4f Tr_inv = Tr.inverse();

        // find ground truth poses.
        std::string poses_file;
        for (int32_t i = 0; i < parts.size() - 4; ++i) {
          poses_file += parts[i].toStdString();
          poses_file += "/";
        }
        poses_file += "poses/" + sequence + ".txt";

        std::cout << "ground truth filename: " << poses_file << std::endl;
        if (rv::FileUtil::exists(poses_file)) {
          std::vector<Eigen::Matrix4f> poses = KITTI::Odometry::loadPoses(poses_file);

          for (uint32_t i = 0; i < poses.size(); ++i) {
            Eigen::Matrix4f pose = poses[i];

            groundtruth_.push_back(Tr_inv * pose * Tr);
          }

          std::cout << groundtruth_.size() << " poses read." << std::endl;
        } else {
          std::cout << "not found." << std::endl;
        }
      } else {
        std::string sequence = parts[parts.size() - 3].toStdString();

        // find ground truth poses.
        std::string poses_file;
        for (int32_t i = 0; i < parts.size() - 4; ++i) {
          poses_file += parts[i].toStdString();
          poses_file += "/";
        }
        poses_file += "poses/" + sequence + ".txt";

        std::cout << "ground truth filename: " << poses_file << std::endl;
        if (rv::FileUtil::exists(poses_file)) {
          std::vector<Eigen::Matrix4f> poses = KITTI::Odometry::loadPoses(poses_file);

          for (uint32_t i = 0; i < poses.size(); ++i) {
            Eigen::Matrix4f pose = poses[i];

            groundtruth_.push_back(pose);
          }

          std::cout << groundtruth_.size() << " poses read." << std::endl;
        } else {
          std::cout << "not found." << std::endl;
        }
      }

      // load the model.
      initilizeKITTIrobot();

      ui_.wCanvas->setGroundtruth(groundtruth_);

      ui_.sldTimeline->setEnabled(reader_->isSeekable());
      ui_.sldTimeline->setMaximum(reader_->count() - 1);
      if (ui_.sldTimeline->value() == 0) setScan(0);  // triggers update of scan.
      ui_.sldTimeline->setValue(0);
    } else if (extension == ".sim") {
      SimulationReader* sr = new SimulationReader(filename.toStdString());

      reader_ = sr;

      uint32_t N = reader_->count();
      precomputed_.resize(N);
      oldPoses_.resize(N);

      for (uint32_t i = 0; i < N; ++i) {
        precomputed_[i] = false;
      }
      if (oldPoses_.size() > 0) oldPoses_[0] = Eigen::Matrix4f::Identity();

      groundtruth_.clear();
      const std::vector<Eigen::Matrix4f>& trajectory = sr->getTrajectory();
      groundtruth_.resize(trajectory.size());
      for (uint32_t i = 0; i < trajectory.size(); ++i) {
        groundtruth_[i] = trajectory[i] * trajectory[0].inverse();
      }
      ui_.wCanvas->setGroundtruth(groundtruth_);

      ui_.sldTimeline->setEnabled(reader_->isSeekable());
      ui_.sldTimeline->setMaximum(reader_->count() - 1);
      if (ui_.sldTimeline->value() == 0) setScan(0);  // triggers update of scan.
      ui_.sldTimeline->setValue(0);
    }
    updatePlaybackControls();
  }

  if (reader_ != nullptr) {
    ui_.wCanvas->setScanCount(reader_->count());
  }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Suma_Ros_Node");
    ros::NodeHandle nh("~");

    readParameters(nh);
    
    ros::Subscriber subLaserScan = nh.subscribe<sensor_msgs::PointCloud2>();
//     ros::Subscriber subImu;
//     
//     ros::Publisher pubCurrentLaserScan_;
//     ros::Publisher pubCurrentLaserScanInfo_;
//     ros::Publisher pubSurfelMap_;
//     ros::Publisher pubSurfelMapInfo_;
//     tf::TransformBroadcaster brTF_;
    
    ros::spin();

    return 0;
}
