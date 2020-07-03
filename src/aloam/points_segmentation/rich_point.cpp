#include <points_segmentation/rich_point.h>

namespace Mobile_Sensing 
{
    
RichPoint& RichPoint::operator=(const RichPoint& other) 
{
    if (this != &other) 
    {   
	// self-assignment check expected
	point_ = other.AsEigenVector();
	intensity_ = other.intensity();
	ring_ = other.ring();
    }
    return *this;
}

RichPoint& RichPoint::operator=(const Eigen::Vector3f& other) 
{
    this->point_ = other;
    return *this;
}

bool RichPoint::operator==(const RichPoint& other) const 
{
    return this->x() == other.x() && this->y() == other.y() &&
           this->z() == other.z() && this->ring() == other.ring() && 
           this->intensity() == other.intensity();
}

}  // namespace depth_clustering
