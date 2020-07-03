// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

std::vector<double> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<double> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

std::vector<double> read_imu_data(const std::string imu_data_path)
{
    std::ifstream imu_data_file;
    imu_data_file.open(imu_data_path.c_str());
    
    std::string line;
    std::getline(imu_data_file, line);
    
    std::string lat, lon, alt, ro, pi, ya, vn, ve, vf, vl, vu, 
		ax, ay, az, af, al, au, wx, wy, wz, wf, wl, wu, 
		pos_accuracy, vel_accuracy, navstat, numsats, posmode, velmode, orimode;

    std::istringstream is(line);
    is >> lat >> lon >> alt >> ro >> pi >> ya >> vn >> ve >> vf >> vl >> vu >> 
	  ax >> ay >> az >> af >> al >> au >> wx >> wy >> wz >> wf >> wl >> wu >> 
	  pos_accuracy >> vel_accuracy >> navstat >> numsats >> posmode >> velmode >> orimode;
	  
    std::vector<double> imu_data_buffer;
    imu_data_buffer.push_back(std::atof(ax.c_str()));
    imu_data_buffer.push_back(std::atof(ay.c_str()));
    imu_data_buffer.push_back(std::atof(az.c_str()));
    imu_data_buffer.push_back(std::atof(wx.c_str()));
    imu_data_buffer.push_back(std::atof(wy.c_str()));
    imu_data_buffer.push_back(std::atof(wz.c_str()));
    imu_data_buffer.push_back(std::atof(ro.c_str()));
    imu_data_buffer.push_back(std::atof(pi.c_str()));
    imu_data_buffer.push_back(std::atof(ya.c_str()));
    
    return imu_data_buffer;
}

double parse_kitti_stamp ( const std::string& s )
{
    // std::cout << "string timestamp is: " << s << std::endl;
    double hour = atof(s.substr(11, 2).c_str()); 
    double min = atof(s.substr(14, 2).c_str());
    double sec = atof(s.substr(17).c_str());
    // std::cout << "hour is: " << hour << ", minute is: " << min << ", second is: " << sec << std::endl;
    
    double timestamp = (hour * 3600) + (min * 60) + sec;
    // std::cout << std::fixed << std::setprecision(10) << "timestamp (num) is: " << timestamp << ", timestamp (string) is: " << s << std::endl;
    return timestamp;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, subfolder, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    n.getParam("subfolder", subfolder);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    ros::Publisher pub_imu_msgs = n.advertise<sensor_msgs::Imu>("/imu/data", 2);

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";
    odomGT.child_frame_id = "/ground_truth";

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";

    std::string lidar_timestamp_path = sequence_number + "/" + subfolder + "/velodyne_points/timestamps.txt";
    std::ifstream lidar_timestamp_file(dataset_folder + lidar_timestamp_path, std::ifstream::in);
    std::cout << "lidar timestamp path: " << lidar_timestamp_path << std::endl;
    
    std::string img_timestamp_path = sequence_number + "/" + subfolder + "/image_00/timestamps.txt";
    std::ifstream img_timestamp_file(dataset_folder + img_timestamp_path, std::ifstream::in);
    std::cout << "timestamp path: " << img_timestamp_path << std::endl;
    
    std::string imu_timestamp_path = sequence_number + "/" + subfolder + "/oxts/timestamps.txt";
    std::ifstream imu_timestamp_file(dataset_folder + imu_timestamp_path, std::ifstream::in);
    std::cout << "imu timestamp path: " << imu_timestamp_path << std::endl;

    std::string ground_truth_path = sequence_number + "/" + subfolder + "/result/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);
    std::cout << "ground truth path: " << ground_truth_path << std::endl;;

    rosbag::Bag bag_out;
    std::stringstream output_file_ss;
    output_file_ss << dataset_folder << "kitti_" + subfolder + ".bag";
    output_file_ss >> output_bag_file;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::string line, line2, line3;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);
    while (  std::getline(lidar_timestamp_file, line) 
	  && std::getline(img_timestamp_file, line2) 
	  && std::getline(imu_timestamp_file, line3) && ros::ok())
    {
        double lidar_timestamp = parse_kitti_stamp(line);
	double img_timestamp = parse_kitti_stamp(line2);
	double imu_timestamp = parse_kitti_stamp(line3);
        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << sequence_number + "/" + subfolder + "/image_00/data/" << std::setfill('0') << std::setw(10) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
        right_image_path << dataset_folder << sequence_number + "/" + subfolder + "/image_01/data/" << std::setfill('0') << std::setw(10) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = std::stof(s);
            }
        }

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        odomGT.header.stamp = ros::Time().fromSec(imu_timestamp);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << sequence_number + "/" + subfolder + "/velodyne_points/data/" 
                        << std::setfill('0') << std::setw(10) << line_num << ".bin";
        std::vector<double> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<double> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(lidar_timestamp);
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

	std_msgs::Header img_header;
	img_header = laser_cloud_msg.header;
	img_header.stamp = ros::Time().fromSec(img_timestamp);
        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(img_header, "mono8", left_image).toImageMsg();
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(img_header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);
	
	
        // read imu data
        std::stringstream imu_data_path;
        imu_data_path << dataset_folder << sequence_number + "/" + subfolder + "/oxts/data/" 
                        << std::setfill('0') << std::setw(10) << line_num << ".txt";
        std::vector<double> imu_data = read_imu_data(imu_data_path.str());
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(imu_data[6], imu_data[7], imu_data[8]);
	
        sensor_msgs::Imu imu_msg;
	imu_msg.linear_acceleration.x = imu_data[0];
	imu_msg.linear_acceleration.y = imu_data[1];
	imu_msg.linear_acceleration.z = imu_data[2];
	imu_msg.angular_velocity.x = imu_data[3];
	imu_msg.angular_velocity.x = imu_data[4];
	imu_msg.angular_velocity.x = imu_data[5];
	imu_msg.orientation.w = quat.w;
	imu_msg.orientation.x = quat.x;
	imu_msg.orientation.y = quat.y;
	imu_msg.orientation.z = quat.z;		
	// std::printf("accX = %f, accY = %f, accZ = %f; angX = %f, angY = %f, angZ = %f.\n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5]);

        imu_msg.header.stamp = ros::Time().fromSec(imu_timestamp);
        imu_msg.header.frame_id = "/camera_init";
        pub_imu_msgs.publish(imu_msg);
	

        if (to_bag)
        {
	    bag_out.write("/imu/data", ros::Time().fromSec(imu_timestamp), imu_msg);
            bag_out.write("/image_left", ros::Time().fromSec(img_timestamp), image_left_msg);
            bag_out.write("/image_right", ros::Time().fromSec(img_timestamp), image_right_msg);
            bag_out.write("/velodyne_points", ros::Time().fromSec(lidar_timestamp), laser_cloud_msg);
            bag_out.write("/path_gt", ros::Time().fromSec(imu_timestamp), pathGT);
            bag_out.write("/odometry_gt", ros::Time().fromSec(imu_timestamp), odomGT);
        }

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}