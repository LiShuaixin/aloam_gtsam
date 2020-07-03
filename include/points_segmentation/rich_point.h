
#ifndef RICH_POINT_H
#define RICH_POINT_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

namespace Mobile_Sensing 
{

class RichPoint 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using AlignedVector = std::vector<RichPoint, Eigen::aligned_allocator<RichPoint>>;

    RichPoint() {}
    explicit RichPoint(float x, float y, float z, float intensity) : point_(Eigen::Vector3f(x, y, z)), intensity_(intensity) {}
    explicit RichPoint(float x, float y, float z, float intensity, uint16_t ring)
	: point_(Eigen::Vector3f(x, y, z)), ring_(ring), intensity_(intensity) {}
    explicit RichPoint(Eigen::Vector3f& eigen_vec) : point_(eigen_vec) {}
    explicit RichPoint(Eigen::Vector3f& eigen_vec, double intensity) : point_(eigen_vec), intensity_(intensity) {}
    ~RichPoint() {}

    inline int ring() const { return ring_; }
    inline float intensity() const { return intensity_; }
    inline float x() const { return point_(0); }
    inline float y() const { return point_(1); }
    inline float z() const { return point_(2); }

    inline uint16_t& ring() { return ring_; }
    inline float& intensity() { return intensity_; }
    inline float& x() { return point_(0); }
    inline float& y() { return point_(1); }
    inline float& z() { return point_(2); }

    inline const Eigen::Vector3f& AsEigenVector() const { return point_; }
    inline Eigen::Vector3f& AsEigenVector() { return point_; }

    inline float DistToSensor2D() const {
      return sqrt(point_(0) * point_(0) + point_(1) * point_(1));
    }

    inline float DistToSensor3D() const {
        return sqrt(point_(0) * point_(0) + point_(1) * point_(1) +
		    point_(2) * point_(2));
    }

    RichPoint& operator=(const RichPoint& other);
    RichPoint& operator=(const Eigen::Vector3f& other);
    bool operator==(const RichPoint& other) const;

private:
    Eigen::Vector3f point_ = Eigen::Vector3f::Zero();
    float intensity_ = 0;
    uint16_t ring_ = 0;
};

}

#endif  // RICH_POINT_H