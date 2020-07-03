#include "points_segmentation/projection_params.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

namespace Mobile_Sensing {

using std::vector;
using std::string;
using std::upper_bound;
using boost::algorithm::starts_with;

ProjectionParams::ProjectionParams ( const int lidar_type ) : lidar_type_(lidar_type)
{
    switch (lidar_type) 
    {
	case 0:
	{
	    this->set_span(SpanParams(-180.0, 180.0, 1024), SpanParams::Direction::HORIZONTAL); // default depth image size is 16 * 870
	    this->set_span(SpanParams(-15.0, 15.0, 16), SpanParams::Direction::VERTICAL);
	    this->fill_cossin();
	    
	    if (!this->valid()) 
	    {
		std::fprintf(stderr, "ERROR: params are not valid!\n");
		return;
	    }
	    break;
	}
	case 1:
	{
	    this->set_span(SpanParams(-180.0, 180.0, 1024), SpanParams::Direction::HORIZONTAL);
	    this->set_span(SpanParams(-30.0, 10.0, 32), SpanParams::Direction::VERTICAL);
	    this->fill_cossin();
	    
	    if (!this->valid()) 
	    {
		std::fprintf(stderr, "ERROR: params are not valid!\n");
		return;
	    }
	    break;
	}
	case 2:
	{
	    this->set_span(SpanParams(-180.0, 180.0, 1024), SpanParams::Direction::HORIZONTAL);
	    
	    SpanParams span_top(-8.5, 2.0, 32);
	    SpanParams span_bottom(-24.87, -8.87, 32);
	    std::vector<SpanParams> spans = {{span_top, span_bottom}};
	    this->set_span(spans, SpanParams::Direction::VERTICAL);
	    
	    this->fill_cossin();
	    
	    if (!this->valid()) 
	    {
		std::fprintf(stderr, "ERROR: params are not valid!\n");
		return;
	    }
	    break;
	}
	case 3:
	{
	    this->set_span(SpanParams(-180.0, 180.0, 1024), SpanParams::Direction::HORIZONTAL);
	    this->set_span(SpanParams(-24.0, 2.0, 64), SpanParams::Direction::VERTICAL);
	    this->fill_cossin();
	    
	    if (!this->valid()) 
	    {
		std::fprintf(stderr, "ERROR: params are not valid!\n");
		return;
	    }
	    break;
	}
    }
}

ProjectionParams::ProjectionParams ( const float& discretization, const int lidar_type /* = FUll_SPHERE = 5 */ ) : lidar_type_(lidar_type)
{
    this->set_span(SpanParams(-180.0, 180.0, discretization), SpanParams::Direction::HORIZONTAL);
    this->set_span(SpanParams(90.0, -90, discretization), SpanParams::Direction::VERTICAL);
    this->fill_cossin();
    
    if (!this->valid()) 
    {
	std::fprintf(stderr, "ERROR: params are not valid!\n");
	return;
    }
}

ProjectionParams::ProjectionParams ( const std::string& path, const int lidar_type /* = VELO_CONFIG = 4 */ ) : lidar_type_(lidar_type)
{
    std::fprintf(stderr, "INFO: Set en_US.UTF-8 locale.\n");
    std::locale::global(std::locale("en_US.UTF-8"));
    std::fprintf(stderr, "INFO: Reading config.\n");
    
    // we need to fill this thing. Parsing text files again. Is that what PhD in
    // Robotics is about?
    std::ifstream file(path.c_str());
    if (!file.is_open()) 
    {
	std::fprintf(stderr, "ERROR: cannot open file: '%s'\n", path.c_str());
	return;
    }
    
    for (std::string line; std::getline(file, line, '\n');) 
    {
	if (starts_with(line, "#")) 
	    std::fprintf(stderr, "INFO: Skipping commentary: \n\t %s\n", line.c_str());
	else 
	{
	    // here we parse the line
	    vector<string> str_angles;
	    boost::split(str_angles, line, boost::is_any_of(";"));
	    if (str_angles.size() < 4) 
	    {
		std::fprintf(stderr, "ERROR: format of line is wrong.\n");
		return;
	    }
	    int cols = std::stoi(str_angles[0]);
	    int rows = std::stoi(str_angles[1]);
	    this->h_span_params_ = SpanParams(std::stod(str_angles[2]), std::stod(str_angles[3]), cols);
	    std::fprintf(stderr, "start:%f, stop:%f, span:%f, step:%f\n", this->h_span_params_.start_angle(),
	        this->h_span_params_.end_angle(), this->h_span_params_.span(), this->h_span_params_.step());

	    // fill the cols spacing
	    for (int c = 0; c < cols; ++c)
		this->col_angles_.push_back(this->h_span_params_.start_angle() + this->h_span_params_.step() * c);

	    // fill the rows
	    this->v_span_params_ = SpanParams(std::stod(str_angles[4]), std::stod(str_angles.back()), rows);
	    // fill the rows with respect to img.cfg spacings
	    for (size_t i = 4; i < str_angles.size(); ++i)
		this->row_angles_.push_back( std::stof(str_angles[i]) );

	    if (this->row_angles_.size() != static_cast<size_t>(rows)) 
	    {
		std::fprintf(stderr, "ERROR: wrong config\n");
		return;
	    }
	}
    }
    
    // fill cos and sin arrays
    this->fill_cossin();
    
    // check validity
    if (!this->valid()) 
    {
	std::fprintf(stderr, "ERROR: the config read was not valid.\n");
	return;
    }
    
    std::fprintf(stderr, "INFO: Params sucessfully read. Rows: %lu, Cols: %lu\n", this->row_angles_.size(), this->col_angles_.size());
}

void ProjectionParams::set_span(const SpanParams& span_params, const SpanParams::Direction& direction) 
{
    std::vector<SpanParams> params_vec = {{span_params}};
    this->set_span(params_vec, direction);
}

void ProjectionParams::set_span(const vector<SpanParams>& span_params, const SpanParams::Direction& direction) 
{
    int num_beams = 0;
    for (const auto& span : span_params)
	num_beams += span.num_beams();
    
    // printf("number of beam = %d.\n", num_beams);
    
    switch (direction) 
    {
	case SpanParams::Direction::HORIZONTAL:
	{
	    h_span_params_ = SpanParams(span_params.front().start_angle(), span_params.back().end_angle(), num_beams);
	    col_angles_ = fill_vector(span_params);
	    break;
	}
	case SpanParams::Direction::VERTICAL:
	{
	    v_span_params_ = SpanParams(span_params.front().start_angle(), span_params.back().end_angle(), num_beams);
	    row_angles_ = fill_vector(span_params);
	    break;
	}
    }
    
    fill_cossin();
}

std::vector<float> ProjectionParams::fill_vector(const SpanParams& span_params) 
{
    std::vector<SpanParams> params_vec = {{span_params}};
    return this->fill_vector(params_vec);
}

std::vector<float> ProjectionParams::fill_vector(const vector<SpanParams>& span_params) 
{
    std::vector<float> res;
    for (const auto span_param : span_params) 
    {	
	float degree = span_param.start_angle();
	for (int i = 0; i < span_param.num_beams(); ++i) 
	{
	    res.push_back(degree);
	    degree += span_param.step();
	}
    }
    return res;
}

bool ProjectionParams::valid() 
{
    bool all_params_valid = v_span_params_.valid() && h_span_params_.valid();
    bool arrays_empty = row_angles_.empty() && col_angles_.empty();
    bool cos_sin_empty = row_angles_sines_.empty() &&
			 row_angles_cosines_.empty() &&
			 col_angles_sines_.empty() && col_angles_cosines_.empty();
    if (!all_params_valid) 
	throw std::runtime_error("Projection parameters invalid.");
    
    if (arrays_empty)
	throw std::runtime_error("Projection parameters arrays not filled.");

    if (cos_sin_empty) 
	throw std::runtime_error("Projection parameters sin and cos arrays not filled.");
    
    return true;
}

const float ProjectionParams::get_angle_from_row(int row) const 
{
    if (row >= 0 && static_cast<size_t>(row) < row_angles_.size())
	return row_angles_[row];
    
    std::fprintf(stderr, "ERROR: row %d is wrong\n", row);
    return 0.0;
}

const float ProjectionParams::get_angle_from_col(int col) const 
{
    int actual_col = col;
    if (col < 0)
	actual_col = col + col_angles_.size();
    else if (static_cast<size_t>(col) >= col_angles_.size())
	actual_col = col - col_angles_.size();

    // everything is normal
    return col_angles_[actual_col];
}

size_t ProjectionParams::get_row_from_angle(const float& angle) const 
{
    return find_closest(row_angles_, angle);
}

size_t ProjectionParams::get_col_from_angle(const float& angle) const 
{
    return find_closest(col_angles_, angle);
}

size_t ProjectionParams::find_closest(const vector<float>& vec, const float& val) 
{
    size_t found = 0;
    if (vec.front() < vec.back()) 
	found = upper_bound(vec.begin(), vec.end(), val) - vec.begin();
    else
	found = vec.rend() - upper_bound(vec.rbegin(), vec.rend(), val);
    
    if (found == 0)
	return found;
  
    if (found == vec.size())
	return found - 1;
    
    auto diff_next = abs(vec[found] - val);
    auto diff_prev = abs(val - vec[found - 1]);
    return diff_next < diff_prev ? found : found - 1;
}

const std::vector<float>& ProjectionParams::row_angle_cosines() const 
{
    return row_angles_cosines_;
}

const std::vector<float>& ProjectionParams::col_angle_cosines() const 
{
    return col_angles_cosines_;
}

const std::vector<float>& ProjectionParams::row_angle_sines() const 
{
    return row_angles_sines_;
}

const std::vector<float>& ProjectionParams::col_angle_sines() const 
{
    return col_angles_sines_;
}

void ProjectionParams::fill_cossin() 
{
    row_angles_sines_.clear();
    row_angles_cosines_.clear();
    for (const auto& angle : row_angles_) 
    {
	auto rad = to_radian(angle);
	row_angles_sines_.push_back( sin(rad) );
	row_angles_cosines_.push_back( cos(rad) );
    }
    
    col_angles_sines_.clear();
    col_angles_cosines_.clear();
    for (const auto& angle : col_angles_) 
    {
	auto rad = to_radian(angle);
	col_angles_sines_.push_back( sin(rad) );
	col_angles_cosines_.push_back( cos(rad) );
    }
}

}  // namespace depth_clustering
