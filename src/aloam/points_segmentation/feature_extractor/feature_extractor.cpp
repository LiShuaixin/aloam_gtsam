#include <points_segmentation/feature_extractor/feature_extractor.h>

namespace Mobile_Sensing 
{
void SmoothnessBasedExtractor::process( ScanProjection* scan_projection
                                      , ImageBasedClusterer<LinearImageLabeler<>>* clusterer
				      , pcl::PointCloud<pcl::PointXYZI>::Ptr points)
{   
    cv::Mat depth_image = scan_projection->depth_image();
    cv::Mat no_ground_image = scan_projection->no_ground_image();
    cv::Mat label_image = scan_projection->ori_label_image();
    std::unordered_map<uint16_t, std::list<size_t>> clusters = clusterer->clusters();
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud = points;
    pcl::copyPointCloud(*points, *laserCloud);
    
    full_points_.clear();
    pcl::copyPointCloud(*points, full_points_);
    
    // reset params
    corner_points_sharp_.clear();
    corner_points_less_sharp_.clear();
    surface_points_flat_.clear();
    surface_points_less_flat_.clear();
    
    // update pointcloud -> remove ground points in the pointcloud
    // float t_test = 0., t_test2 = 0., t_test3 = 0., t_test4 = 0.;
    for (int row = 0; row < depth_image.rows; ++row) 
    {
	std::vector<PointProperty*> scan_info;
	
	// 获取一行点云数据并标识是否为地面点
	std::list<uint16_t> erase_label;
	TicToc t_label;
	for (int col = 0; col < depth_image.cols; ++col) 
	{
	    int ground_flag = 1;
	    
	    const auto& point_container = scan_projection->at(row, col);
	    if (point_container.is_empty()) // this is ok, just continue, nothing interesting here, no points.
		continue;

	    uint16_t label = no_ground_image.at<uint16_t>(row, col);
	    if (label > 0.01)// this is a default label meaning that it's not a ground pixel, skip
	    {
		ground_flag = 0;
		
		label = label_image.at<uint16_t>(row, col);
		if(!erase_label.empty())
		{
		    std::list<uint16_t>::iterator it = std::find(erase_label.begin(), erase_label.end(), label);
		    if (it != erase_label.end())
			continue;
		}
	
		if(clusters.find(label) == clusters.end())
		{  
		    erase_label.push_back(label);
		    continue;
		}
		
	    }
	    
	    for (const auto& point_idx : point_container.points())
	    {
		// update flag_ground and point_idx for the point_info
		PointProperty* point_info = new PointProperty();
		point_info->set_flag_ground(ground_flag);
		point_info->set_point_idx(point_idx);
		point_info->set_flag_segmentation(label);
		
		scan_info.push_back(point_info); // push the point_info into the scan_info
	    }
	}
	// t_test += t_label.toc();
	    
	int num_points_in_line = scan_info.size();
	if(num_points_in_line == 0)
	    continue;
	
	// 点云根据遮挡关系剔除
	// TicToc t_block;
	set_scan_buffer(scan_info, laserCloud);
	// t_test2 += t_block.toc();
		
	for (int j = 0; j < extraction_params().feature_regions_; j++) 
	{  
	    int sp = (extraction_params().curvature_region_ * (extraction_params().feature_regions_ - j)
                      + (num_points_in_line - extraction_params().curvature_region_) * j) / extraction_params().feature_regions_; // start index for scan line(without the first 5 points)	    
	    int ep = (extraction_params().curvature_region_ * (extraction_params().feature_regions_ - 1 - j)
                      + (num_points_in_line - extraction_params().curvature_region_) * (j + 1)) / extraction_params().feature_regions_ - 1;

	    // skip empty regions
	    if (ep <= sp)
		continue;

	    int regionSize = ep - sp + 1;

	    // 分段计算点云曲率并根据smoothness排序(越大越不平坦)->按照smoothness由小至大的顺序排列
	    // TicToc t_cal;
	    set_region_buffer(sp, ep, scan_info, laserCloud);
	    // t_test3 += t_cal.toc();
	
	    // extract corner features
	    int largestPickedNum = 0;
	    // TicToc t_insert;
	    for (int k = regionSize - 1; k > 0 && largestPickedNum < extraction_params().max_corner_less_sharp_; k--) 
	    {
		int idx = scan_info[sp+k]->get_point_idx(); // the index of the greatest smoothness point
		int picked = scan_info[sp+k]->get_flag_picked(); 
		int ground = scan_info[sp+k]->get_flag_ground();
		float curvature = scan_info[sp+k]->get_curvature();
		PointLabel label = scan_info[sp+k]->get_label();

		if (picked == 0 && curvature > extraction_params().surface_curvature_threshold_ && ground == 0)
		{

		    largestPickedNum++;
		    if (largestPickedNum <= extraction_params().max_corner_sharp_)
		    {
			scan_info[sp+k]->set_label( CORNER_SHARP );
			corner_points_sharp_.push_back(std::make_pair(row,idx));
		    } 
		    else 
		    {
			scan_info[sp+k]->set_label(CORNER_LESS_SHARP);
		    }
		    corner_points_less_sharp_.push_back(std::make_pair(row,idx));
		    
		    // mark neighbors as picked
		    mark_as_picked( (sp+k), scan_info, laserCloud );
		}		
	    }
	    

	    // extract flat surface features
	    int smallestPickedNum = 0;
	    for (int k = 0; k < regionSize && smallestPickedNum < extraction_params().max_surface_flat_; k++) 
	    {
		int idx = scan_info[sp+k]->get_point_idx(); // the index of the greatest smoothness point
		int picked = scan_info[sp+k]->get_flag_picked(); 
		int ground = scan_info[sp+k]->get_flag_ground();
		float curvature = scan_info[sp+k]->get_curvature();
		PointLabel label = scan_info[sp+k]->get_label();

		if (picked == 0 && curvature < extraction_params().surface_curvature_threshold_ && ground == 1) 
		{

		    smallestPickedNum++;
		    scan_info[sp+k]->set_label(SURFACE_FLAT);
		    surface_points_flat_.push_back(std::make_pair(row,idx));
		    surface_points_less_flat_.push_back(std::make_pair(row,idx));
		    
		    // mark neighbors as picked
		    mark_as_picked( (sp+k), scan_info, laserCloud );
		}		
	    }

	    // extract less flat surface features
	    int count = 0;
	    for (int k = 0; k < regionSize; k++)
	    {
		int idx = scan_info[sp+k]->get_point_idx();
		
		if (scan_info[sp+k]->get_label() == SURFACE_LESS_FLAT){ 
		    if(count == 3)
		    {
			surface_points_less_flat_.push_back(std::make_pair(row,idx));
			count = 0;
	            }
	            count++;
		}
	    }
	    // t_test4 += t_insert.toc();
	}
    }
    // std::printf("------label ground point time %f ms.\n", t_test);
    // std::printf("------remove blocked point time %f ms.\n", t_test2);
    // std::printf("------sort points time %f ms.\n", t_test3);
    // std::printf("------insert points time %f ms.\n", t_test4);
}

void SmoothnessBasedExtractor::mark_as_picked(// const PointProperty& scan_info, 
					      // vector< int >& scan_neighbor_picked,
					      const int& object_idx_in_scan,
			                      std::vector<PointProperty*>& scan_info,
					      pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud )
{
    int cloud_idx = scan_info[object_idx_in_scan]->get_point_idx();         
    
    // scan_neighbor_picked[scanline_idx] = 1;
    scan_info[object_idx_in_scan]->set_flag_picked(1);

    for (int i = 1; i <= extraction_params().curvature_region_; i++) 
    {
	if (calc_squared_diff(laserCloud->points[cloud_idx + i], laserCloud->points[cloud_idx + i - 1]) > 0.05 || (cloud_idx + i) > laserCloud->size())
	    break;

	// scan_neighbor_picked[scanline_idx + i] = 1;
	scan_info[object_idx_in_scan + i]->set_flag_picked(1);
    }

    for (int i = 1; i <= extraction_params().curvature_region_; i++) 
    {
	if (calc_squared_diff(laserCloud->points[cloud_idx - i], laserCloud->points[cloud_idx - i - 1]) > 0.05 || (cloud_idx - i - 1) < 0)
	    break;

	// scan_neighbor_picked[scanline_idx - i] = 1;
	scan_info[object_idx_in_scan - i]->set_flag_picked(1);
    }
}


void SmoothnessBasedExtractor::set_region_buffer(const int& startIdx, const int& endIdx, 
						 std::vector<PointProperty*>& scan_info,
						 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud)
{    
    // resize buffers
    int regionSize = endIdx - startIdx + 1;   

    // calculate point curvatures and reset sort indices
    float pointWeight = -2 * extraction_params().curvature_region_;

    for (int i = startIdx; i <= endIdx; i++) 
    {
	float diffX = pointWeight * laserCloud->points[scan_info[i]->get_point_idx()].x;
	float diffY = pointWeight * laserCloud->points[scan_info[i]->get_point_idx()].y;
	float diffZ = pointWeight * laserCloud->points[scan_info[i]->get_point_idx()].z;

	for (int j = 1; j <= extraction_params().curvature_region_; j++) 
	{
	    diffX += laserCloud->points[scan_info[i+j]->get_point_idx()].x 
	           + laserCloud->points[scan_info[i-j]->get_point_idx()].x;
	    diffY += laserCloud->points[scan_info[i+j]->get_point_idx()].y 
	           + laserCloud->points[scan_info[i-j]->get_point_idx()].y;
	    diffZ += laserCloud->points[scan_info[i+j]->get_point_idx()].z 
	           + laserCloud->points[scan_info[i-j]->get_point_idx()].z;
	}
	
	// update curvature, label and scanline_idx for scan_info
	scan_info[i]->set_curvature(diffX * diffX + diffY * diffY + diffZ * diffZ);
	scan_info[i]->set_label(SURFACE_LESS_FLAT);
	scan_info[i]->set_scanline_idx(i);
    }

    std::vector<PointProperty*>::iterator it_start = scan_info.begin() + startIdx - 1;
    std::vector<PointProperty*>::iterator it_end = scan_info.begin() + endIdx - 1;
    std::sort(it_start,it_end,compare_smoothness);  
}


void SmoothnessBasedExtractor::set_scan_buffer(std::vector<PointProperty*>& scan_info,
					       pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud )
{
    int scan_size = scan_info.size(); // num of points in scan line
    // std::cout << "[DEBUG EXTRACTOR] Set Scan Buffer:: scan_size = " << scan_size << std::endl;
    
    // mark unreliable points as picked
    for (int i = extraction_params().curvature_region_; i < scan_size - extraction_params().curvature_region_ - 1; i++)
    {	
	const pcl::PointXYZI& previousPoint = (laserCloud->points[ scan_info[i - 1]->get_point_idx() ]);
	const pcl::PointXYZI& point = (laserCloud->points[ scan_info[i]->get_point_idx() ]);
	const pcl::PointXYZI& nextPoint = (laserCloud->points[ scan_info[i + 1]->get_point_idx() ]);
	// std::printf("[DEBUG EXTRACTOR] Set Scan Buffer:: %dth point = [%f,%f,%f], pre = [%f,%f,%f], next = [%f,%f,%f].\n", i, point.x, point.y, point.z,
	//                previousPoint.x, previousPoint.y, previousPoint.z, nextPoint.x, nextPoint.y, nextPoint.z);

	float diffNext = calc_squared_diff(nextPoint, point);
        
	// 判断遮挡关系->update flag_picked for the scan_info
	if (diffNext > 0.1) 
	{
	    float depth1 = calc_point_distance(point);
	    float depth2 = calc_point_distance(nextPoint);

	    
	    if (depth1 > depth2) 
	    {
		float sin_theta = std::sqrt( calc_squared_diff(nextPoint, point, depth2 / depth1) ) / depth2;

		if (sin_theta < 0.1) 
		{
		    for(int k = 0; k <= extraction_params().curvature_region_; k++)
			scan_info[i - k]->set_flag_picked(1);

		    continue;
		}
	    } 
	    else 
	    {
		float sin_theta = std::sqrt( calc_squared_diff(point, nextPoint, depth1 / depth2) ) / depth1;

		if (sin_theta < 0.1) 
		{
		    for(int k = 0; k <= extraction_params().curvature_region_; k++)
			scan_info[i + k + 1]->set_flag_picked(1);
		}
	    }
	}

	float diffPrevious = calc_squared_diff(point, previousPoint);
	float dis = calc_squared_point_distance(point);

	// 判断入射角情况
	if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis)
	    scan_info[i]->set_flag_picked(1);
    }

}

void SmoothnessBasedExtractor::get_corner_points_sharp(pcl::PointCloud< pcl::PointXYZI >::Ptr points)
{
    if(full_points_.empty())
    {
	std::printf("[ERROR] No point cloud.\n");
	return;
    }
    
    std::vector<std::pair<int,int>> vec = corner_points_sharp_;   
    
    /// No need to sort?
    //std::sort(sharp_corner.begin(), sharp_corner.end(), compare_point_id()); // sort vector elements
    
    std::vector<std::pair<int,int>>::const_iterator it = vec.begin();
    for( ; it != vec.end(); it++)
    {
        int idx = it->second;
	pcl::PointXYZI pt = full_points_.points[idx];
	points->push_back(pt);
    }
}

void SmoothnessBasedExtractor::get_corner_points_less_sharp(pcl::PointCloud< pcl::PointXYZI >::Ptr points)
{
    if(full_points_.empty())
    {
	std::printf("[ERROR] No point cloud.\n");
	return;
    }
    
    std::vector<std::pair<int,int>> vec = corner_points_less_sharp_;   
    
    /// No need to sort?
    //std::sort(sharp_corner.begin(), sharp_corner.end(), compare_point_id()); // sort vector elements
    
    std::vector<std::pair<int,int>>::const_iterator it = vec.begin();
    for( ; it != vec.end(); it++)
    {
        int idx = it->second;
	pcl::PointXYZI pt = full_points_.points[idx];
	points->push_back(pt);
    }

}

void SmoothnessBasedExtractor::get_surf_points_flat(pcl::PointCloud< pcl::PointXYZI >::Ptr points)
{
    if(full_points_.empty())
    {
	std::printf("[ERROR] No point cloud.\n");
	return;
    }
    
    std::vector<std::pair<int,int>> vec = surface_points_flat_;   
    
    /// No need to sort?
    //std::sort(sharp_corner.begin(), sharp_corner.end(), compare_point_id()); // sort vector elements
    
    std::vector<std::pair<int,int>>::const_iterator it = vec.begin();
    for( ; it != vec.end(); it++)
    {
        int idx = it->second;
	pcl::PointXYZI pt = full_points_.points[idx];
	points->push_back(pt);
    }
}

void SmoothnessBasedExtractor::get_surf_points_less_flat(pcl::PointCloud< pcl::PointXYZI >::Ptr points)
{
    if(full_points_.empty())
    {
	std::printf("[ERROR] No point cloud.\n");
	return;
    }
    
    std::vector<std::pair<int,int>> vec = surface_points_less_flat_;   
    
    /// No need to sort?
    //std::sort(sharp_corner.begin(), sharp_corner.end(), compare_point_id()); // sort vector elements
    
    std::vector<std::pair<int,int>>::const_iterator it = vec.begin();
    for( ; it != vec.end(); it++)
    {
        int idx = it->second;
	pcl::PointXYZI pt = full_points_.points[idx];
	points->push_back(pt);
    }
}

}  // namespace Mobile_Sensing
