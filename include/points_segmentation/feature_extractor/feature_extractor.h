#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <vector>
#include <cmath>
#include <iostream>
#include <inttypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/impl/io.hpp>

#include <opencv/cv.h> 

#include <points_segmentation/feature_extractor//depth_ground_remover.h>
#include <points_segmentation/feature_extractor/image_based_clusterer.h>
#include <aloam_velodyne/tic_toc.h>

namespace Mobile_Sensing 
{
/** Point label options. */
enum PointLabel
{
    CORNER_SHARP = 2,       ///< sharp corner point
    CORNER_LESS_SHARP = 1,  ///< less sharp corner point
    SURFACE_LESS_FLAT = 0,  ///< less flat surface point
    SURFACE_FLAT = -1       ///< flat surface point
};
  
/** Scan Extraction configuration parameters. */
class ExtractionParams
{
public:
    ExtractionParams(const float& scan_period = 0.1,
		       const int& feature_regions = 6,
		       const int& curvature_region = 5,
		       const int& max_corner_sharp = 6,
		       const int& max_surface_flat = 12,
		       const float& less_flat_filter_size = 0.2,
		       const float& surface_curvature_threshold = 0.1,
		       const float& min_cluster_size = 20,
		       const float& max_cluster_size = 25000)
                     : scan_period_(scan_period)
		     , feature_regions_(feature_regions)
		     , curvature_region_(curvature_region)
		     , max_corner_sharp_(max_corner_sharp)
		     , max_corner_less_sharp_(10 * max_corner_sharp)
		     , max_surface_flat_(max_surface_flat)
		     , less_flat_filter_size_(less_flat_filter_size)
		     , surface_curvature_threshold_(surface_curvature_threshold)
		     , min_cluster_size_(min_cluster_size)
		     , max_cluster_size_(max_cluster_size)
		     {}

    /** The time per scan. */
    float scan_period_;

    /** The size of the IMU history state buffer. */
    // int imuHistorySize;

    /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
    int feature_regions_;

    /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
    int curvature_region_;

    /** The maximum number of sharp corner points per feature region. */
    int max_corner_sharp_;

    /** The maximum number of less sharp corner points per feature region. */
    int max_corner_less_sharp_;

    /** The maximum number of flat surface points per feature region. */
    int max_surface_flat_;

    /** The voxel size used for down sizing the remaining less flat surface points. */
    float less_flat_filter_size_;

    /** The curvature threshold below / above a point is considered a flat / corner point. */
    float surface_curvature_threshold_;
    
    /** The segmentation threshold below / above a point is considered a unvalidate / validate point. */
    int min_cluster_size_;
    
    /** The segmentation threshold below / above a point is considered a validate / unvalidate point. */
    int max_cluster_size_;
};   

class PointProperty
{
public:
    PointProperty( const float& curvature = -1.0,
	           const PointLabel& region_label = SURFACE_LESS_FLAT,
		   const size_t& point_idx = -1,
		   const size_t& scanline_idx = -1,
		   const int& flag_picked = 0,
		   const int& flag_ground = -1,
		   const int& flag_segmentation = -1
 		)
                 : curvature_(curvature)
                 , region_label_(region_label)
		 , point_idx_(point_idx)
                 , scanline_idx_(scanline_idx)
                 , flag_picked_(flag_picked)
                 , flag_ground_(flag_ground)
		 , flag_segmentation_(flag_segmentation)
		 {}

    inline void set_curvature( const float& curvature ) { curvature_ = curvature; };
    inline float get_curvature() const { return curvature_; }
    
    inline void set_label( const PointLabel& region_label ) { region_label_ = region_label; };
    inline PointLabel get_label() const { return region_label_; }
    
    inline void set_point_idx( const size_t& point_idx ) { point_idx_ = point_idx; };
    inline size_t get_point_idx() const { return point_idx_; }
    
    inline void set_scanline_idx( const size_t& scanline_idx ) { scanline_idx_ = scanline_idx; };
    inline size_t get_scanline_idx() const { return scanline_idx_; }
    
    inline void set_flag_picked( const int& flag_picked ) { flag_picked_ = flag_picked; };
    inline int get_flag_picked() const { return flag_picked_; }
    
    inline void set_flag_ground( const int& flag_ground ) { flag_ground_ = flag_ground; };
    inline int get_flag_ground() const { return flag_ground_; }
    
    inline void set_flag_segmentation( const uint16_t& flag_segmentation ) { flag_segmentation_ = flag_segmentation; };
    inline uint16_t get_flag_segmentation() const { return flag_segmentation_; }
    
private:
    float curvature_;      ///< point curvature buffer
    
    PointLabel region_label_;     ///< point label buffer
    
    int point_idx_;   ///< sorted region indices based on point curvature
    
    int scanline_idx_;
    
    int flag_picked_;     ///< flag if neighboring point was already picked
    
    int flag_ground_;
    
    uint16_t flag_segmentation_;
};

/**
 * @brief      A class to extract feature points, including removing ground based upon depth image,
 *             classfying points into different clusters based upon angle difference, 
 *             and extracting surf/flat points based upon smoothness
 * @details    Given a depth image, image clusters and image config, this class should select surf
 *             and flat points return the them.
 */
class AbstractExtractor
{
public:
    /** 
     * @brief    Constructor with default params.
     *
     */
    explicit AbstractExtractor(const ExtractionParams& extraction_params = ExtractionParams()) 
    : extraction_params_(extraction_params) {}
    
    ExtractionParams extraction_params() { return extraction_params_; }
    
    
private:
    ExtractionParams extraction_params_; 
   
};

class SmoothnessBasedExtractor : public AbstractExtractor
{
public:
    /** 
     * @brief    Constructor with default params.
     *
     */
    explicit SmoothnessBasedExtractor(const ExtractionParams& extraction_params = ExtractionParams())
    : AbstractExtractor(extraction_params)
    {}
    
    virtual ~SmoothnessBasedExtractor() {}
    
    /** 
     * @brief    Process a new cloud as a set of scanlines.
     *
     * @param    Scan the scan need to be processed
     */
    void process( ScanProjection* scan_projection
                , ImageBasedClusterer<LinearImageLabeler<>>* clusterer
		, pcl::PointCloud<pcl::PointXYZI>::Ptr points);
    
    /** 
     * @brief    Set up region buffers for the specified point range.
     *
     * @param    scan_line_points_idx the line scan index
     * @param    scan the scan
     */
    void set_region_buffer(const int& startIdx, const int& endIdx, 
			   std::vector<PointProperty*>& scan_info,
			   pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud);
    
    /** 
     * @brief    Set up scan buffers for the specified point range.
     *
     * @param    scan_line_points_idx the line scan index
     * @param    scan the scan
     */
    void set_scan_buffer(std::vector<PointProperty*>& scan_info,
			 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud );
    
    /** 
     * @brief    Mark a point and its neighbors as picked.
     *
     * @details  This method will mark neighboring points within the curvature region 
     *           as picked, as long as they remain within close distance to each other.
     *
     * @param    cloud_idx the index of the picked point in the full resolution cloud
     * @param    scan_idx the index of the picked point relative to the current scan
     */
    void mark_as_picked(// const PointProperty& scan_info, 
	             // std::vector<int>& scan_neighbor_picked,
			const int& object_idx_in_scan,
			std::vector<PointProperty*>& scan_info,
			pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud );   
    
    void get_corner_points_sharp(pcl::PointCloud<pcl::PointXYZI>::Ptr points);
    void get_corner_points_less_sharp(pcl::PointCloud<pcl::PointXYZI>::Ptr points);
    void get_surf_points_flat(pcl::PointCloud<pcl::PointXYZI>::Ptr points);
    void get_surf_points_less_flat(pcl::PointCloud<pcl::PointXYZI>::Ptr points);
    
    template <typename PointT>
    float calc_squared_diff(const PointT& a, const PointT& b)
    {
	float diffX = a.x - b.x;
	float diffY = a.y - b.y;
	float diffZ = a.z - b.z;

	return diffX * diffX + diffY * diffY + diffZ * diffZ;
    }

    template <typename PointT>
    float calc_squared_diff(const PointT& a, const PointT& b, const float& wb)
    {
	float diffX = a.x - b.x * wb;
	float diffY = a.y - b.y * wb;
	float diffZ = a.z - b.z * wb;

	return diffX * diffX + diffY * diffY + diffZ * diffZ;
    }
	
    template <typename PointT>
    float calc_point_distance(const PointT& p)
    {
	return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    template <typename PointT>
    float calc_squared_point_distance(const PointT& p)
    {
	return p.x * p.x + p.y * p.y + p.z * p.z;
    }
    
    bool compare_point_id(const std::pair< int, int > a, const std::pair< int, int > b) { return a.second < b.second; } 
    bool compare_scan_line(const std::pair< int, int > a, const std::pair< int, int > b) { return a.first < b.first; }
    static bool compare_smoothness(const PointProperty* a, const PointProperty* b) { return ( a->get_curvature() < b->get_curvature() ); }
    
private:  
    std::vector<std::pair<int,int> > corner_points_less_sharp_;
    std::vector<std::pair<int,int> > corner_points_sharp_; 
    std::vector<std::pair<int,int> > surface_points_less_flat_;
    std::vector<std::pair<int,int> > surface_points_flat_;
    
    pcl::PointCloud<pcl::PointXYZI> full_points_;
};

}  // namespace Mobile_Sensing

#endif  // FEATURE_EXTRACTOR_H