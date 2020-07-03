#ifndef IMAGE_BASED_CLUSTERER_H
#define IMAGE_BASED_CLUSTERER_H

#include <chrono>
#include <ctime>
#include <map>
#include <opencv/cv.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <points_segmentation/image_labelers/radians.h>
#include <points_segmentation/image_labelers/diff_helpers/diff_factory.h>
#include <points_segmentation/image_labelers/linear_image_labeler.h>
#include <points_segmentation/scan_projection.h>

using namespace depth_clustering;
    
namespace Mobile_Sensing 
{
    
/**
 * @brief      Class for abstract clusterer.
 */
class AbstractClusterer
{
public:
    /**
     * @brief      Construct a clusterer.
     *
     * @param[in]  cluster_tollerance  The cluster tollerance
     * @param[in]  min_cluster_size    The minimum cluster size
     * @param[in]  max_cluster_size    The maximum cluster size
     * @param[in]  skip                Only cluster every skip cloud
     */
    explicit AbstractClusterer(double cluster_tollerance = 0.2,
				uint16_t min_cluster_size = 20,
				uint16_t max_cluster_size = 25000,
				uint16_t skip = 10)
	: cluster_tollerance_(cluster_tollerance),
	  min_cluster_size_(min_cluster_size),
	  max_cluster_size_(max_cluster_size),
	  skip_(skip),
	  counter_(0) {}
    virtual ~AbstractClusterer() {}

protected:
    double cluster_tollerance_;
    uint16_t min_cluster_size_;
    uint16_t max_cluster_size_;
    uint16_t skip_;
    uint32_t counter_;
};

/**
 * @brief      Class for image based clusterer.
 *
 * @tparam     LabelerT  A Labeler class to be used for labeling.
 */
template <typename LabelerT>
class ImageBasedClusterer : public AbstractClusterer 
{

public:
    /**
     * @brief      Construct an image-based clusterer.
     *
     * @param[in]  angle_tollerance  The angle tollerance to separate objects
     * @param[in]  min_cluster_size  The minimum cluster size to send
     * @param[in]  max_cluster_size  The maximum cluster size to send
     */
    explicit ImageBasedClusterer(float angle_tollerance = 8.0, uint16_t min_cluster_size = 60, uint16_t max_cluster_size = 25000)
    : AbstractClusterer(0.0, min_cluster_size, max_cluster_size),
      counter_(0),
      angle_tollerance_(angle_tollerance) {}

    virtual ~ImageBasedClusterer() {}

    /**
     * @brief      Sets the difference type.
     *
     * @param[in]  diff_type  The difference type
     */
    void set_diff_type(DiffFactory::DiffType diff_type) { diff_type_ = diff_type; }


    /**
     * @brief      Gets called when clusterer receives a cloud to cluster
     *
     * @param[in]  cloud      The cloud to cluster
     * @param[in]  sender_id  The sender identifier
     */
    void process( ScanProjection* scan_projection )
    {
	// generate a projection from a point cloud
	Radians angle_tollerance(depth_clustering::Radians::IsRadians{}, (angle_tollerance_*M_PI / 180.) );
	LabelerT image_labeler(scan_projection->no_ground_image(), *(scan_projection->projection_params()), angle_tollerance);
	image_labeler.ComputeLabels(diff_type_);
	const cv::Mat* labels_ptr = image_labeler.GetLabelImage();
	
	cv::Mat labels_color_image = image_labeler.LabelsToColor(*labels_ptr); // CV_8UC3
	// std::cout << "[DEBUG INFO] label_image.row = " << labels_color_image.rows << ", label_image.col = " << labels_color_image.cols << std::endl;
	scan_projection->clone_label_image(labels_color_image);
	scan_projection->clone_ori_label_image(*labels_ptr);

	// create 3d clusters from image labels
	std::unordered_map<uint16_t, std::list<size_t>> clusters;
	for (int row = 0; row < labels_ptr->rows; ++row) 
	{
	    for (int col = 0; col < labels_ptr->cols; ++col) 
	    {
		const auto& point_container = scan_projection->at(row, col);
		if (point_container.is_empty()) // this is ok, just continue, nothing interesting here, no points.
		    continue;
		
		uint16_t label = labels_ptr->at<uint16_t>(row, col);
		if (label < 1) // this is a default label, skip
		    continue;
		
		for (const auto& point_idx : point_container.points()) 
		    clusters[label].push_back(point_idx);
	    }
	}
	// std::cout << "[DEBUG FeatureExtractor] before erase, cluster.size = " << clusters.size() << std::endl;

	// filter out unfitting clusters
	std::vector<uint16_t> labels_to_erase;
	for (const auto& kv : clusters) 
	{
	    const auto& cluster = kv.second;
	    if (cluster.size() < this->min_cluster_size_ ||
		cluster.size() > this->max_cluster_size_) 
		labels_to_erase.push_back(kv.first);
	}
	for (auto label : labels_to_erase)
	    clusters.erase(label);
	
	clusters_ = clusters;
    }

    inline std::unordered_map<uint16_t, std::list<size_t>> clusters() { return clusters_; }
private:
    int counter_;
    float angle_tollerance_;
    
    std::unordered_map<uint16_t, std::list<size_t>> clusters_;

    DiffFactory::DiffType diff_type_ = DiffFactory::DiffType::NONE;
};

}// namespace Mobile_Sensing

#endif  // IMAGE_BASED_CLUSTERER_H