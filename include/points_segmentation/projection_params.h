#ifndef PROJECTION_PARAMS_H
#define PROJECTION_PARAMS_H

#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <utility>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace Mobile_Sensing 
{
//NOTE all float degree should use degree instead of radian
/**
 * @brief 保存lidar参数， 包括线数、水平和垂直方向的起始和终止角、步长
 * @record HDL64 10Hz 水平角分辨率 = 360/2083
 *         HDL32 10Hz 水平角分辨率 = 360/2169
 *         VLP16 10Hz 水平角分辨率 = 360/1800
 */ 
class SpanParams 
{
public:
    /**
    * Enum for the direction of the span.
    */
    enum class Direction { HORIZONTAL, VERTICAL };

    SpanParams() {}
    SpanParams( const float& start_angle, const float& end_angle, const float& step ) 
    {
	start_angle_ = start_angle;
	end_angle_ = end_angle;
	step_ = step;
	num_beams_ = std::floor((end_angle - start_angle) / step);
	span_ = std::abs(end_angle - start_angle);
    }

    SpanParams( const float& start_angle, const float& end_angle, int num_beams ) 
    {
	start_angle_ = start_angle;
	end_angle_ = end_angle;
	num_beams_ = num_beams;
	step_ = (end_angle - start_angle) / num_beams;
	span_ = std::abs(end_angle - start_angle);
    }

    const float& start_angle() const { return start_angle_; }
    const float& end_angle() const { return end_angle_; }
    const float& step() const { return step_; }
    const float& span() const { return span_; }
    int num_beams() const { return num_beams_; }

    bool valid() const { return num_beams_ > 0 && span_ > 0.0; }

private:
    
    float start_angle_ = 0.0;
    float end_angle_ = 0.0;
    float step_ = 0.0;
    float span_ = 0.0;
    int num_beams_ = 0;
};

/**
 * @brief      Class for projection parameters.
 */
class ProjectionParams 
{
public:

enum class Set { COLS, ROWS };
    
    ProjectionParams() {}
    
    /**
    * @brief      Default parameters for 16/32/64/64_equal beam Velodyne
    */
    ProjectionParams( const int lidar_type );
    
    /**
    * @brief      Default parameters for Velodyne from config file
    */
    ProjectionParams( const std::string& path, const int lidar_type = 4 );
    
    /**
    * @brief      Default parameters to cover full sphere
    */
    ProjectionParams( const float& discretization, const int lidar_type = 5 );
    
    ~ProjectionParams() {} 
    
    /**
    * @brief      Set the angle span in a given direction.
    *
    * @param[in]  span_params  The span parameters packad into ::SpanParams.
    * @param[in]  direction    The direction. Must be one of
    *                          SpanParams::Direction.
    */
    void set_span(const SpanParams& span_params, const SpanParams::Direction& direction);

    /**
    * @brief      Set the angle spans in a given direction.
    *
    * @param[in]  span_params  The span parameters packad into ::SpanParams
    * @param[in]  direction    The direction. Must be one of
    *                          SpanParams::Direction.
    */
    void set_span(const std::vector<SpanParams>& span_params, const SpanParams::Direction& direction);

    inline const float& vertical_start_angle() const { return v_span_params_.start_angle(); }
    inline const float& vertical_end_angle() const { return v_span_params_.end_angle(); }
    inline const float& vertical_span() const { return v_span_params_.span(); }

    inline const float& horizontal_start_angle() const { return h_span_params_.start_angle(); }
    inline const float& horizontal_end_angle() const { return h_span_params_.end_angle(); }
    inline const float& horizontal_span() const { return h_span_params_.span(); }
    
    inline size_t rows() const { return row_angles_.size(); }
    inline size_t cols() const { return col_angles_.size(); }
    inline size_t size() const { return rows() * cols(); }
    
    inline int lidar_type() const { return lidar_type_; }
    
    inline void depth_to_color ( cv::Mat& color, const cv::Mat& depth, const double max, const double min )
    { 
	cv::Mat grayImage; 
	double alpha = 255.0 / (max - min); 
	depth.convertTo(grayImage, CV_8UC1, alpha, -alpha * min);
	cv::applyColorMap(grayImage, color, cv::COLORMAP_HSV);
    }

    inline float to_degree ( const float& rad )
    {
	float degree = rad * 180.0f / M_PI;
	return degree;
    }

    inline float to_radian ( const float& degree )
    {
	float rad = degree * M_PI / 180.0f;
	return rad;
    }

    /**
    * @brief      Get angle from row
    *
    * @param[in]  row   The row
    *
    * @return     Angle in radians
    */
    const float get_angle_from_row(int row) const;

    /**
    * @brief      Get angle from col
    *
    * @param[in]  col   The col
    *
    * @return     Angle in radians
    */
    const float get_angle_from_col(int col) const;

    /**
    * @brief      Get row number from angle
    *
    * @param[in]  angle  The angle
    *
    * @return     Row number
    */
    size_t get_row_from_angle(const float& angle) const;

    /**
    * @brief      Get col number from angle
    *
    * @param[in]  angle  The angle
    *
    * @return     Col number
    */
    size_t get_col_from_angle(const float& angle) const;

    const std::vector<float>& row_angle_cosines() const;
    const std::vector<float>& col_angle_cosines() const;
    const std::vector<float>& row_angle_sines() const;
    const std::vector<float>& col_angle_sines() const;

    bool valid();

private:
    // compute degree for each scan line
    std::vector<float> fill_vector(const SpanParams& span_params);
    std::vector<float> fill_vector(const std::vector<SpanParams>& span_params);

    // find the closest one in vec for val
    static size_t find_closest(const std::vector<float>& vec, const float& val);

    // 
    void fill_cossin();
    
    int lidar_type_;

    SpanParams v_span_params_;
    SpanParams h_span_params_;

    std::vector<float> col_angles_;
    std::vector<float> row_angles_;

    std::vector<float> col_angles_sines_;
    std::vector<float> col_angles_cosines_;

    std::vector<float> row_angles_sines_;
    std::vector<float> row_angles_cosines_;
};

}

#endif  // PROJECTION_PARAMS_H
