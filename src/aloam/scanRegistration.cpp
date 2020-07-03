// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <points_segmentation/projection_params.h>
#include <points_segmentation/scan_projection.h>
#include <points_segmentation/feature_extractor/depth_ground_remover.h>
#include <points_segmentation/feature_extractor/image_based_clusterer.h>
#include <points_segmentation/feature_extractor/feature_extractor.h>
#include <points_segmentation/image_labelers/diff_helpers/diff_factory.h>
#include <points_segmentation/image_labelers/linear_image_labeler.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using std::atan2;
using std::cos;
using std::sin;

bool segmentFeatures = false;
const double scanPeriod = 0.1;

const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
ros::Publisher pub_depth_img, pub_segmentation_img, pub_angle_img;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1; 

std::string SCAN_TOPIC = "/velodyne_points";
std::string IMU_TOPIC = "/imu/data";

int LIDAR_TYPE = 0; // 0 for VLP-16, 1 for HDL-32, 2 for HDL-64, 3 for HLD-64 equal

int COUNT = 1;
double SUM_TIME = 0.0;

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    if(laserCloudIn.empty())
    {
	ROS_WARN("laser scan at timestamp: %f is empty!!", laserCloudMsg->header.stamp.toSec());
	return;
    }
    
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            std::printf("[ERROR] wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }
    // std::printf("prepare time %f \n", t_prepare.toc());
    
    pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
    
    if(segmentFeatures) 
    {
	/// \note Points segmentation
	TicToc t_pts, t_build;
	
	// create prejection param object
	ProjectionParams* projection_params = new ProjectionParams(LIDAR_TYPE);
	
	// create depth_ground_remover
	int smooth_window_size = 9;
	float ground_remove_angle = 7.;   
	DepthGroundRemover* depth_ground_remover_ = new DepthGroundRemover( *projection_params, ground_remove_angle, smooth_window_size );

	// create image_based_clusterer
	int min_cluster_size = 20;
	int max_cluster_size = 100000;
	float angle_tollerance = 10.;
	ImageBasedClusterer<LinearImageLabeler<>>* clusterer = new ImageBasedClusterer<LinearImageLabeler<>>(angle_tollerance, min_cluster_size, max_cluster_size);
	clusterer->set_diff_type(DiffFactory::DiffType::ANGLES);
	
	// create feature_extractor
	SmoothnessBasedExtractor* feature_extractor = new SmoothnessBasedExtractor();
	// std::printf("====build tool time %f ms.\n", t_build.toc());
	
	// project points to depth image
	TicToc t_projection;
	ScanProjection* scan_projection = new ScanProjection(projection_params);
	scan_projection->init_from_points(laserCloud);
	// std::printf("====scan projection time %f ms.\n", t_projection.toc());
	
	// preprocess raw pointcloud
	TicToc t_remove;
	depth_ground_remover_->process(scan_projection);
	// std::printf("====ground points remove time %f ms.\n", t_remove.toc());
	
	TicToc t_cluster;
	std::unordered_map<uint16_t, std::list<size_t>> clusters;
	clusterer->process(scan_projection); 
	// std::printf("====points cluster time %f ms.\n", t_cluster.toc());
	
	TicToc t_extraction;
	feature_extractor->process(scan_projection, clusterer, laserCloud);
	// std::printf("====feature extraction time %f ms.\n", t_extraction.toc());	
	
	cornerPointsSharp->clear();
	cornerPointsLessSharp->clear();
	surfPointsFlat->clear();
	surfPointsLessFlat->clear();
	
	feature_extractor->get_corner_points_sharp(cornerPointsSharp);
	feature_extractor->get_corner_points_less_sharp(cornerPointsLessSharp);
	feature_extractor->get_surf_points_flat(surfPointsFlat);
	feature_extractor->get_surf_points_less_flat(surfPointsLessFlat);
	// std::printf("size of cornerPointsSharp = %d, size of cornerPointsLessSharp = %d, size of surfPointsFlat = %d, size of surfPointsLessFlat = %d.\n"
	//            , cornerPointsSharp->size(), cornerPointsLessSharp->size(), surfPointsFlat->size(), surfPointsLessFlat->size());
	//std::printf("seperate points time %f \n", t_pts.toc());
	
	// pub preprocessed images
	cv_bridge::CvImage cvi;
	cvi.header.stamp = laserCloudMsg->header.stamp;
	cvi.header.frame_id = "/velodyne";
	cvi.encoding = "bgr8";
	
	cvi.image = scan_projection->label_image();
	sensor_msgs::Image segmentationImage2;
	cvi.toImageMsg(segmentationImage2);
	pub_segmentation_img.publish(segmentationImage2);
	
	cvi.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	cvi.image = scan_projection->depth_image();
	sensor_msgs::Image depthImage2;
	cvi.toImageMsg(depthImage2);
	pub_depth_img.publish(depthImage2);
	// printf("scan's depth image size: %d * %d.\n", mapper->cur_scan_.scan_projection_->depth_image().rows, mapper->cur_scan_.scan_projection_->depth_image().cols);
	// since I'd like to show the depth image without ground, the frame drawer should be put after ground remover
	
	cvi.image = depth_ground_remover_->smoothed_angle_image();
	sensor_msgs::Image angleImage2;
	cvi.toImageMsg(angleImage2);
	pub_angle_img.publish(angleImage2);
    }
    else
    {
	for (int i = 5; i < cloudSize - 5; i++)
	{ 
	    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
	    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
	    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

	    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
	    cloudSortInd[i] = i;
	    cloudNeighborPicked[i] = 0;
	    cloudLabel[i] = 0;
	}


	TicToc t_pts;

	cornerPointsSharp->clear();
	cornerPointsLessSharp->clear();
	surfPointsFlat->clear();
	surfPointsLessFlat->clear();

	float t_q_sort = 0;
	for (int i = 0; i < N_SCANS; i++)
	{
	    if( scanEndInd[i] - scanStartInd[i] < 6)
		continue;
	    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
	    for (int j = 0; j < 6; j++)
	    {
		int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
		int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

		TicToc t_tmp;
		std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
		t_q_sort += t_tmp.toc();

		int largestPickedNum = 0;
		for (int k = ep; k >= sp; k--)
		{
		    int ind = cloudSortInd[k]; 

		    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
		    {

			largestPickedNum++;
			if (largestPickedNum <= 2)
			{                        
			    cloudLabel[ind] = 2;
			    cornerPointsSharp->push_back(laserCloud->points[ind]);
			    cornerPointsLessSharp->push_back(laserCloud->points[ind]);
			}
			else if (largestPickedNum <= 20)
			{                        
			    cloudLabel[ind] = 1; 
			    cornerPointsLessSharp->push_back(laserCloud->points[ind]);
			}
			else
			{
			    break;
			}

			cloudNeighborPicked[ind] = 1; 

			for (int l = 1; l <= 5; l++)
			{
			    float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
			    float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
			    float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
			    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
			    {
				break;
			    }

			    cloudNeighborPicked[ind + l] = 1;
			}
			for (int l = -1; l >= -5; l--)
			{
			    float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
			    float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
			    float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
			    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
			    {
				break;
			    }

			    cloudNeighborPicked[ind + l] = 1;
			}
		    }
		}

		int smallestPickedNum = 0;
		for (int k = sp; k <= ep; k++)
		{
		    int ind = cloudSortInd[k];

		    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
		    {

			cloudLabel[ind] = -1; 
			surfPointsFlat->push_back(laserCloud->points[ind]);

			smallestPickedNum++;
			if (smallestPickedNum >= 4)
			{ 
			    break;
			}

			cloudNeighborPicked[ind] = 1;
			for (int l = 1; l <= 5; l++)
			{ 
			    float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
			    float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
			    float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
			    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
			    {
				break;
			    }

			    cloudNeighborPicked[ind + l] = 1;
			}
			for (int l = -1; l >= -5; l--)
			{
			    float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
			    float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
			    float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
			    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
			    {
				break;
			    }

			    cloudNeighborPicked[ind + l] = 1;
			}
		    }
		}

		for (int k = sp; k <= ep; k++)
		{
		    if (cloudLabel[k] <= 0)
		    {
			surfPointsLessFlatScan->push_back(laserCloud->points[k]);
		    }
		}
	    }

	    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
	    pcl::VoxelGrid<PointType> downSizeFilter;
	    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
	    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
	    downSizeFilter.filter(surfPointsLessFlatScanDS);

	    *surfPointsLessFlat += surfPointsLessFlatScanDS;
	}
// 	std::printf("sort q time %f \n", t_q_sort);
// 	std::printf("seperate points time %f \n", t_pts.toc());
    }

    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/velodyne";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/velodyne";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/velodyne";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/velodyne";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/velodyne";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);  

    // pub each scam
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "/velodyne";
            pubEachScan[i].publish(scanMsg);
        }
    }

    // std::printf("scan registration time %f ms *************\n", t_whole.toc());
    SUM_TIME += t_whole.toc();
    // std::printf("mean scan registration time %f ms for %d scan *************\n", SUM_TIME/COUNT, COUNT);
    COUNT++;
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    nh.param<int>("scan_line", N_SCANS, 16);

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);
    
    nh.param<std::string>("scan_topic", SCAN_TOPIC, "/velodyne_points");
    nh.param<std::string>("imu_topic", IMU_TOPIC, "/imu/data");
    
    nh.param<int>("lidar_type", LIDAR_TYPE, 0);
    
    nh.param<bool>("segment_features", segmentFeatures, false);

    std::printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        std::printf("[ERROR] only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(SCAN_TOPIC, 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    pub_depth_img = nh.advertise<sensor_msgs::Image>("/depth_image", 50);

    pub_segmentation_img = nh.advertise<sensor_msgs::Image>("/segmentation_image", 50);

    pub_angle_img = nh.advertise<sensor_msgs::Image>("/angle_image", 50);

    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    return 0;
}
