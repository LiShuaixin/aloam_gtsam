#ifndef DEPTH_GROUND_REMOVER_H
#define DEPTH_GROUND_REMOVER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <algorithm>
#include <chrono>

#include <points_segmentation/projection_params.h>
#include <points_segmentation/scan_projection.h>
#include <points_segmentation/image_labelers/radians.h>
#include <points_segmentation/image_labelers/linear_image_labeler.h>
#include <points_segmentation/image_labelers/diff_helpers/angle_diff.h>
#include <points_segmentation/image_labelers/diff_helpers/simple_diff.h>

namespace Mobile_Sensing 
{

/**
 * @brief      A class to quickly classify objects, remove ground based upon depth image
 * @details    Given a depth image and image config, this class should remove the
 *             ground and return the new depth image with no ground.
 * @param      params  projection params
 */
class DepthGroundRemover
{
public:
    explicit DepthGroundRemover(const ProjectionParams& proj_params, const float& ground_remove_angle, int window_size = 5) : 
             projection_params_(proj_params),
             window_size_(window_size),
             ground_remove_angle_(ground_remove_angle) 
    {
	int row = proj_params.rows();
        int col = proj_params.cols();
	
	angle_image_ = cv::Mat(row, col, CV_8UC3, cv::Scalar(0,0,0));
	smoothed_angle_image_ = cv::Mat(row, col, CV_8UC3, cv::Scalar(0,0,0)); 	
    }
             
    virtual ~DepthGroundRemover() {}

    /**
    * @brief      main process func
    * @details    receiving a depth image, then removing ground from it
    *
    * @param      depth_image  32 bit depth image
    */
    void process( ScanProjection* scan_projection );
    // void process_lego( ScanProjection* scan_projection );
    
    inline cv::Mat angle_image() { return angle_image_; }
    inline cv::Mat smoothed_angle_image() { return smoothed_angle_image_; }

protected:
    /**
    * @brief      Zero out all pixels that belong to ground
    *
    * @param[in]  image        Input depth image
    * @param[in]  angle_image  The angle image
    * @param[in]  threshold    angle threshold
    *
    * @return     depth image with 0 instead of ground pixels
    */
    cv::Mat zero_out_ground(const cv::Mat& image, const cv::Mat& angle_image, const float& threshold) const;

    cv::Mat zero_out_ground_BFS(const cv::Mat& image, const cv::Mat& angle_image, const float& threshold, int kernel_size) const;

    /**
    * @brief      create a help image with angle in radians written for each
    *             pixel
    *
    * @param      depth_image  [input depth image]
    * @return     [32 bit float image with angle in radians written in every
    *             pixel]
    */
    cv::Mat create_angle_image(const cv::Mat& depth_image);

    /**
    * @brief      Get kernel for Savitsky-Golay filter
    * @details    Get a column filter to process an image filled with data with
    *             Savitsky-Golay filter
    *
    * @param      window_size  size of the kernel
    * @return     column Mat kernel
    */
    cv::Mat get_savitsky_golay_kernel(int window_size) const;
    cv::Mat get_uniform_kernel(int window_size, int type = CV_32F) const;

    /**
    * @brief      Apply Savitsky-Golay smoothing to a column
    * @param      column  [A column of an angle image]
    * @return     [a smoothed column]
    */

    cv::Mat apply_savitsky_golay_smoothing(const cv::Mat& column, int window_size);
    /**
    * @brief      Get line angle
    * @details    Given two depth values and their angles compute the angle of
    *             the line that they spawn in the sensor frame.
    *
    * @param      depth_image  [32 bit float image]
    * @param      col          [current column]
    * @param      row_curr     [current row]
    * @param      row_neigh    [neighbor row]
    * @return     [angle of the line (degree)]
    */
    float get_line_angle(const cv::Mat& depth_image, int col, int row_curr, int row_neigh);

    /**
    * @brief      Repair zeros in the depth image
    *
    * @param[in]  depth_image  The depth image
    *
    * @return     depth image with repaired values
    */
    cv::Mat repair_depth_image(const cv::Mat& no_ground_image, int step, float depth_threshold);

    cv::Mat repair_depth_image(const cv::Mat& depth_image);

protected:
    
    ProjectionParams projection_params_;
  
    int window_size_ = 5;
  
    float ground_remove_angle_ = 5.0;
  
    float eps_ = 0.001f;
    
    cv::Mat angle_image_;
    cv::Mat smoothed_angle_image_;
};

}  // namespace Mobile_Sensing

#endif  // DEPTH_GROUND_REMOVER_H
