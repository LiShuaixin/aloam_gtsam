#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include "SuMa/visualizer/preprocessWorker.h"
#include "SuMa/util/parameters.h"
#include "SuMa/rv/PrimitiveParameters.h"
#include "SuMa/core/SurfelMapping.h"
#include "aloam_velodyne/tic_toc.h"

using namespace std;

class PreprocessNode
{
public:
    PreprocessNode(ros::NodeHandle nh) : nh_(nh)
    {
	/// \note Read params and create ParameterList
	readParameters(nh);
	
	rv::ParameterList paramList;
	paramList.insert(rv::IntegerParameter("data_width", DEPTH_COL));
	paramList.insert(rv::IntegerParameter("data_height", DEPTH_ROW));
	paramList.insert(rv::FloatParameter("data_fov_up", FOV_UP));
	paramList.insert(rv::FloatParameter("data_fov_down", FOV_DOWN));
	paramList.insert(rv::BooleanParameter("filter_vertexmap", FILTER_VERTEXMAP));
	paramList.insert(rv::FloatParameter("bilateral_sigma_space", BILATERAL_SPACE));
	paramList.insert(rv::FloatParameter("bilateral_sigma_range", BILATERAL_RANGE));
	paramList.insert(rv::BooleanParameter("use_filtered_vertexmap", USW_FILTERED_VERTEXMAP));
	paramList.insert(rv::FloatParameter("min_depth", MIN_DEPTH));
	paramList.insert(rv::FloatParameter("max_depth", MAX_DEPTH));
	paramList.insert(rv::FloatParameter("min_yaw", MIN_YAW));
	paramList.insert(rv::FloatParameter("max_yaw", MAX_YAW));
	std::printf("insert params into the list!!\n");
	
	std::shared_ptr<SurfelMapping> fusion = std::shared_ptr<SurfelMapping>(new SurfelMapping(paramList));

        // window.initialize(fusion, params);

	ros::Subscriber subLaserScan = nh_.subscribe<sensor_msgs::PointCloud2>(SCAN_TOPIC, 100, &PreprocessNode::laserScanHandler, this);  
    
	pubVertexImg_ = nh_.advertise<sensor_msgs::PointCloud>("/vertex_img", 1000);
	pubNormalImg_ = nh_.advertise<sensor_msgs::PointCloud>("/normal_img", 1000);
	pubRestart_ = nh_.advertise<std_msgs::Bool>("/restart",1000);
	pubVertexImgVisual_ = nh_.advertise<sensor_msgs::Image>("/vertex_img_visual", 1000);
	pubNormalImgVisual_ = nh_.advertise<sensor_msgs::Image>("/normal_img_visual", 1000);
    }
    
    ~PreprocessNode() {}
    
    void laserScanHandler(const sensor_msgs::PointCloud2ConstPtr& laserScanMsgIn)
    {
	/// \note detect unstable laser scan stream
	if (laserScanMsgIn->header.stamp.toSec() - lastScanTime_ > 1.0 || laserScanMsgIn->header.stamp.toSec() < lastScanTime_)
	{
	    ROS_WARN("laser scan discontinue! reset the preprocessor!");
	    firstScanFlag_ = true; 
	    lastScanTime_ = 0;
	    pubCount_ = 1;
	    std_msgs::Bool restartFlag;
	    restartFlag.data = true;
	    pubRestart_.publish(restartFlag);
	    return;
	}
	
	lastScanTime_ = laserScanMsgIn->header.stamp.toSec();
	
	/// \note Convert laserScanMsg to rv::Laserscan
	pcl::PointCloud<pcl::PointXYZI> pclLaserScanIn;
	pcl::fromROSMsg(*laserScanMsgIn, pclLaserScanIn);
	if(pclLaserScanIn.empty())
	{
	    ROS_WARN("laser scan at timestamp: %f is empty!!", laserScanMsgIn->header.stamp.toSec());
	    return;
	}
	int num_points = pclLaserScanIn.size();
	std::printf("the size of input scan is %f.", num_points);
	
	rv::Laserscan laserScanIn;   
	std::vector<rv::Point3f>& points = laserScanIn.points();
	std::vector<float>& remissions = laserScanIn.remissions();

	points.resize(num_points);
	remissions.resize(num_points);

	float max_remission = 0;

	for (uint32_t i = 0; i < num_points; ++i) {
	    points[i].x() = pclLaserScanIn.points[i].x;
	    points[i].y() = pclLaserScanIn.points[i].y;
	    points[i].z() = pclLaserScanIn.points[i].z;
	    remissions[i] = pclLaserScanIn.points[i].intensity;
	    max_remission = std::max(remissions[i], max_remission);
	}

	for (uint32_t i = 0; i < num_points; ++i) {
	    remissions[i] /= max_remission;
	}
	
	/// \note Laser scan preprocess
	// preprocessWorker_.processScan(laserScanIn);
	std::printf("preprocess done!!\n");
	
	
	/// \note Publish the result depth image
	
    }
    
private:
    // std::shared_ptr< PreprocessWorker > preprocessWorker_;
    std::shared_ptr<SurfelMapping> fusion_;

    queue<sensor_msgs::PointCloud2ConstPtr> laserScanBuf_;

    ros::NodeHandle nh_;
    ros::Publisher pubVertexImg_, pubNormalImg_;
    ros::Publisher pubVertexImgVisual_, pubNormalImgVisual_;
    ros::Publisher pubRestart_;

    double firstScanTime_;
    int pubCount_ = 1;
    bool firstScanFlag_ = true;
    double lastScanTime_ = 0;
    bool initPub_ = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "preprocess");
    ros::NodeHandle nh("~");
    
    PreprocessNode PN(nh);

    ROS_INFO("\033[1;32m---->\033[0m Data Preprocessing Started.");

    ros::spin();

    return 0;
}
