#ifndef SCAN_PROJECTION_H
#define SCAN_PROJECTION_H

#include <list>
#include <memory>
#include <stdexcept>
#include <vector>
#include <math.h>

#include <tbb/tbb.h>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <points_segmentation/projection_params.h>
#include <points_segmentation/rich_point.h>

namespace Mobile_Sensing 
{

class ProjectionParams;

/**
 * @brief  class for cloud projection.
 */
class ScanProjection 
{
public:    
    class PointContainer;
    
    using PointColumn = std::vector<PointContainer>;
    using PointMatrix = std::vector<PointColumn>;
    

public:
    ScanProjection();
    
    ScanProjection( ProjectionParams* params);
    
    ScanProjection(const ScanProjection& scan_proj);
    
    ~ScanProjection() {}

    /**
    * @brief      Initialize from 3d points.
    *
    * @param[in]  points  The points
    */
    void init_from_points( pcl::PointCloud<pcl::PointXYZI>::Ptr points);

    const cv::Mat& depth_image() const;
    cv::Mat& depth_image();
    
    const cv::Mat& label_image() const;
    cv::Mat& label_image();
    
    const cv::Mat& ori_label_image() const;
    cv::Mat& ori_label_image();
    
    const cv::Mat& no_ground_image() const;
    cv::Mat& no_ground_image();
    
    inline void clone_depth_image(const cv::Mat& image) { depth_image_ = image.clone(); }
    inline void clone_label_image(const cv::Mat& image) { label_image_ = image.clone(); }
    inline void clone_ori_label_image(const cv::Mat& image) { ori_label_image_ = image.clone(); }
    inline void clone_no_ground_image(const cv::Mat& image) { no_ground_image_ = image.clone(); }

    inline size_t rows() const { return projection_params_->rows(); }
    
    inline size_t cols() const { return projection_params_->cols(); }
    
    inline size_t size() const { return projection_params_->size(); }
    
    inline ProjectionParams* projection_params() const { return projection_params_; }
    
    inline const PointContainer& at(const size_t row, const size_t col) const { return data_[col][row]; }
    
    inline PointContainer& at(const size_t row, const size_t col) { return data_[col][row]; }
    
    inline const PointMatrix& matrix() const { return data_; }
    
    inline std::vector<float> corrections() const { return corrections_; }

    /**
    * @brief      Set corrections for systematic error in a dataset (see
    *             notebooks in the repo)
    *
    * @param[in]  corrections  A vector of correction in depth for every beam.
    */
    inline void set_corrections(const std::vector<float>& corrections) { corrections_ = corrections; }
    
    /**
    * @brief      Check if where we store data is valid.
    *
    * @param[in]  image  The image to check
    */
    void check_image(const cv::Mat& image);

    /**
    * @brief      Check if where we store data is valid.
    *
    * @param[in]  points  The points to check
    */
    void check_scan( pcl::PointCloud<pcl::PointXYZI>::Ptr points);

    /**
    * @brief      Unproject a point from depth image coordinate
    *
    * @param[in]  image  A depth image
    * @param[in]  row    A row in the image
    * @param[in]  col    A col in the image
    *
    * @return     { description_of_the_return_value }
    */
    pcl::PointXYZI unproject_points(const cv::Mat& image, const int row, const int col) const;


    /**
    * @brief      Fix systematic error. See notebooks in the repo for details.
    */
    void fix_depth_systematic_error();

protected:
    // just stores addresses of the points. Does not own them.
    PointMatrix data_;

    ProjectionParams* projection_params_;

    cv::Mat depth_image_;
    cv::Mat label_image_;
    cv::Mat ori_label_image_;
    cv::Mat no_ground_image_;

    std::vector<float> corrections_;
};

/**
 * @brief      Class for point container.
 */
class ScanProjection::PointContainer 
{
public:
    PointContainer();
    inline bool is_empty() const { return points_.empty(); }
    inline std::list<size_t>& points() { return points_; }
    inline const std::list<size_t>& points() const { return points_; }

private:
    std::list<size_t> points_;// points on each pixel
};

}  // namespace depth_clustering

#endif  // SCAN_PROJECTION_H
