#include "removert/Removerter.h"

inline float rad2deg(float radians) 
{ 
    return radians * 180.0 / M_PI; 
}

inline float deg2rad(float degrees) 
{ 
    return degrees * M_PI / 180.0; 
}

void Removerter::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    cout << "TODO" << endl;
}

void fsmkdir(std::string _path)
{
    if (!fs::is_directory(_path) || !fs::exists(_path)) 
        fs::create_directories(_path); // create src folder
} //fsmkdir

Removerter::Removerter()
{
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/os1_points", 5, &Removerter::cloudHandler, this, ros::TransportHints().tcpNoDelay());

    // voxelgrid generates warnings frequently, so verbose off + ps. recommend to use octree (see makeGlobalMap)
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR); 
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    nh.param<int>("removert/num_nn_points_within", kNumKnnPointsToCompare, 3); // using higher, more strict static 
    nh.param<float>("removert/dist_nn_points_within", kScanKnnAndMapKnnAvgDiffThreshold, 0.1); // using smaller, more strict static 

    if( save_pcd_directory_.substr(save_pcd_directory_.size()-1, 1) != std::string("/") )
        save_pcd_directory_ = save_pcd_directory_ + "/";
    fsmkdir(save_pcd_directory_);
    scan_static_save_dir_ = save_pcd_directory_ + "scan_static"; fsmkdir(scan_static_save_dir_);
    scan_dynamic_save_dir_ = save_pcd_directory_ + "scan_dynamic"; fsmkdir(scan_dynamic_save_dir_);
    map_static_save_dir_ = save_pcd_directory_ + "map_static"; fsmkdir(map_static_save_dir_);
    map_dynamic_save_dir_ = save_pcd_directory_ + "map_dynamic"; fsmkdir(map_dynamic_save_dir_);

    allocateMemory();

    // ros pub for visual debug
    scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("removert/scan_single_local", 1);
    global_scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("removert/scan_single_global", 1);

    original_map_local_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("removert/original_map", 10);

    curr_map_local_publisher_ =  nh.advertise<sensor_msgs::PointCloud2> ("removert/curr_map", 1);
    static_map_local_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("removert/static_map", 1);
    dynamic_map_local_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("removert/dynamic_map", 1);

    static_curr_scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("removert/scan_single_local_static", 1);
    dynamic_curr_scan_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("removert/scan_single_local_dynamic", 1);

} // ctor


void Removerter::allocateMemory()
{
    map_global_orig_.reset(new pcl::PointCloud<PointType>());

    map_global_curr_.reset(new pcl::PointCloud<PointType>());
    map_local_curr_.reset(new pcl::PointCloud<PointType>());

    map_global_curr_static_.reset(new pcl::PointCloud<PointType>());
    map_global_curr_dynamic_.reset(new pcl::PointCloud<PointType>());

    map_subset_global_curr_.reset(new pcl::PointCloud<PointType>());

    kdtree_map_global_curr_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_scan_global_curr_.reset(new pcl::KdTreeFLANN<PointType>());

    //
    map_global_orig_->clear();
    map_global_curr_->clear();
    map_local_curr_->clear();
    map_global_curr_static_->clear();
    map_global_curr_dynamic_->clear();
    map_subset_global_curr_->clear();

} // allocateMemory


Removerter::~Removerter(){}


void Removerter::parseValidScanInfo( void )
{
    int num_valid_parsed {0};
    float movement_counter {0.0}; 

    for(int curr_idx=0; curr_idx < int(sequence_scan_paths_.size()); curr_idx++) 
    {
        // check the scan idx within the target idx range 
        if(curr_idx > end_idx_ || curr_idx < start_idx_) {
            curr_idx++;
            continue;
        }

        // check enough movement occured (e.g., parse every 2m)
        if(use_keyframe_gap_) {
            if( remainder(num_valid_parsed, keyframe_gap_) != 0 ) {
                num_valid_parsed++;
                continue;
            }
        }
        if(use_keyframe_meter_) {
            if( 0 /*TODO*/ ) {
                // TODO using movement_counter
            }
        }

        // save the info (reading scan bin is in makeGlobalMap) 
        sequence_valid_scan_paths_.emplace_back(sequence_scan_paths_.at(curr_idx));
        sequence_valid_scan_names_.emplace_back(sequence_scan_names_.at(curr_idx));

        scan_poses_.emplace_back(sequence_scan_poses_.at(curr_idx)); // used for local2global
        scan_inverse_poses_.emplace_back(sequence_scan_inverse_poses_.at(curr_idx)); // used for global2local

        // 
        num_valid_parsed++;
    }

    if(use_keyframe_gap_) {
        ROS_INFO_STREAM("\033[1;32m Total " << sequence_valid_scan_paths_.size()
            << " nodes are used from the index range [" << start_idx_ << ", " << end_idx_ << "]" 
            << " (every " << keyframe_gap_ << " frames parsed)\033[0m");
    }
} // parseValidScanInfo


void Removerter::readValidScans( void )
// for target range of scan idx 
{
    const int cout_interval {10};
    int cout_counter {0};

    for(auto& _scan_path : sequence_valid_scan_paths_) 
    {
        // read bin files and save  
        pcl::PointCloud<PointType>::Ptr points (new pcl::PointCloud<PointType>); // pcl::PointCloud Ptr is a shared ptr so this points will be automatically destroyed after this function block (because no others ref it).
        if( isScanFileKITTIFormat_ ) {
            readBin(_scan_path, points); // For KITTI (.bin)
        } else {
            pcl::io::loadPCDFile<PointType> (_scan_path, *points); // saved from SC-LIO-SAM's pcd binary (.pcd)
        }

        // pcdown
        pcl::VoxelGrid<PointType> downsize_filter;
        downsize_filter.setLeafSize(kDownsampleVoxelSize, kDownsampleVoxelSize, kDownsampleVoxelSize);
        downsize_filter.setInputCloud(points);

        pcl::PointCloud<PointType>::Ptr downsampled_points (new pcl::PointCloud<PointType>);
        downsize_filter.filter(*downsampled_points);

        // save downsampled pointcloud
        scans_.emplace_back(downsampled_points);

        // cout for debug  
        cout_counter++;
        if (remainder(cout_counter, cout_interval) == 0) {
            cout << _scan_path << endl;
            cout << "Read a pointcloud with " << points->points.size() << " points." << endl;
            cout << "downsample the pointcloud: " << downsampled_points->points.size() << " points." << endl;
            cout << " ... (display every " << cout_interval << " readings) ..." << endl;
        }
    }
    cout << endl;
} // readValidScans


std::pair<cv::Mat, cv::Mat> Removerter::map2RangeImg(const pcl::PointCloud<PointType>::Ptr& _scan, 
                      const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                      const std::pair<int, int> _rimg_size)
{
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;
    
    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;

    // @ range image initizliation 
    cv::Mat rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(kFlagNoPOINT)); // float matrix, save range value 
    cv::Mat rimg_ptidx = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32SC1, cv::Scalar::all(0)); // int matrix, save point (of global map) index

    // @ points to range img 
    int num_points = _scan->points.size();
    #pragma omp parallel for num_threads(kNumOmpCores)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx)
    {   
        PointType this_point = _scan->points[pt_idx];
        SphericalPoint sph_point = cart2sph(this_point);

        // @ note about vfov: e.g., (+ V_FOV/2) to adjust [-15, 15] to [0, 30]
        // @ min and max is just for the easier (naive) boundary checks. 
        int lower_bound_row_idx {0}; 
        int lower_bound_col_idx {0};
        int upper_bound_row_idx {kNumRimgRow - 1}; 
        int upper_bound_col_idx {kNumRimgCol - 1};
        int pixel_idx_row = int(std::min(std::max(std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV/float(2.0))) / (kVFOV - float(0.0)))), float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV/float(2.0))) / (kHFOV - float(0.0)))), float(lower_bound_col_idx)), float(upper_bound_col_idx)));

        float curr_range = sph_point.r;

        // @ Theoretically, this if-block would have race condition (i.e., this is a critical section), 
        // @ But, the resulting range image is acceptable (watching via Rviz), 
        // @      so I just naively applied omp pragma for this whole for-block (2020.10.28)
        // @ Reason: because this for loop is splited by the omp, points in a single splited for range do not race among them, 
        // @         also, a point A and B lied in different for-segments do not tend to correspond to the same pixel, 
        // #               so we can assume practically there are few race conditions.     
        // @ P.S. some explicit mutexing directive makes the code even slower ref: https://stackoverflow.com/questions/2396430/how-to-use-lock-in-openmp
        if ( curr_range < rimg.at<float>(pixel_idx_row, pixel_idx_col) ) {
            rimg.at<float>(pixel_idx_row, pixel_idx_col) = curr_range;
            rimg_ptidx.at<int>(pixel_idx_row, pixel_idx_col) = pt_idx;
        }
    }

    return std::pair<cv::Mat, cv::Mat>(rimg, rimg_ptidx);
} // map2RangeImg


cv::Mat Removerter::scan2RangeImg(const pcl::PointCloud<PointType>::Ptr& _scan, 
                      const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                      const std::pair<int, int> _rimg_size)
{
    const float kVFOV = _fov.first;
    const float kHFOV = _fov.second;
    
    const int kNumRimgRow = _rimg_size.first;
    const int kNumRimgCol = _rimg_size.second;
    // cout << "rimg size is: [" << _rimg_size.first << ", " << _rimg_size.second << "]." << endl;

    // @ range image initizliation 
    cv::Mat rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(kFlagNoPOINT)); // float matrix

    // @ points to range img 
    int num_points = _scan->points.size();
    #pragma omp parallel for num_threads(kNumOmpCores)
    for (int pt_idx = 0; pt_idx < num_points; ++pt_idx)
    {   
        PointType this_point = _scan->points[pt_idx];
        SphericalPoint sph_point = cart2sph(this_point);

        // @ note about vfov: e.g., (+ V_FOV/2) to adjust [-15, 15] to [0, 30]
        // @ min and max is just for the easier (naive) boundary checks. 
        int lower_bound_row_idx {0}; 
        int lower_bound_col_idx {0};
        int upper_bound_row_idx {kNumRimgRow - 1}; 
        int upper_bound_col_idx {kNumRimgCol - 1};
        int pixel_idx_row = int(std::min(std::max(std::round(kNumRimgRow * (1 - (rad2deg(sph_point.el) + (kVFOV/float(2.0))) / (kVFOV - float(0.0)))), float(lower_bound_row_idx)), float(upper_bound_row_idx)));
        int pixel_idx_col = int(std::min(std::max(std::round(kNumRimgCol * ((rad2deg(sph_point.az) + (kHFOV/float(2.0))) / (kHFOV - float(0.0)))), float(lower_bound_col_idx)), float(upper_bound_col_idx)));

        float curr_range = sph_point.r;

        // @ Theoretically, this if-block would have race condition (i.e., this is a critical section), 
        // @ But, the resulting range image is acceptable (watching via Rviz), 
        // @      so I just naively applied omp pragma for this whole for-block (2020.10.28)
        // @ Reason: because this for loop is splited by the omp, points in a single splited for range do not race among them, 
        // @         also, a point A and B lied in different for-segments do not tend to correspond to the same pixel, 
        // #               so we can assume practically there are few race conditions.     
        // @ P.S. some explicit mutexing directive makes the code even slower ref: https://stackoverflow.com/questions/2396430/how-to-use-lock-in-openmp
        if ( curr_range < rimg.at<float>(pixel_idx_row, pixel_idx_col) ) {
            rimg.at<float>(pixel_idx_row, pixel_idx_col) = curr_range;
        }
    }

    return rimg;
} // scan2RangeImg


void Removerter::mergeScansWithinGlobalCoord( 
        const std::vector<pcl::PointCloud<PointType>::Ptr>& _scans, 
        const std::vector<Eigen::Matrix4d>& _scans_poses,
        pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save )
{
    // NOTE: _scans must be in local coord
    for(std::size_t scan_idx = 0 ; scan_idx < _scans.size(); scan_idx++)
    {
        auto ii_scan = _scans.at(scan_idx); // pcl::PointCloud<PointType>::Ptr
        auto ii_pose = _scans_poses.at(scan_idx); // Eigen::Matrix4d 

        // local to global (local2global)
        pcl::PointCloud<PointType>::Ptr scan_global_coord(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*ii_scan, *scan_global_coord, kSE3MatExtrinsicLiDARtoPoseBase);
        pcl::transformPointCloud(*scan_global_coord, *scan_global_coord, ii_pose);

        // merge the scan into the global map
        *_ptcloud_to_save += *scan_global_coord;
    }
} // mergeScansWithinGlobalCoord


void Removerter::octreeDownsampling(const pcl::PointCloud<PointType>::Ptr& _src, pcl::PointCloud<PointType>::Ptr& _to_save)
{
    pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree( kDownsampleVoxelSize );
    octree.setInputCloud(_src);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
    octree.getVoxelCentroids(centroids);

    // init current map with the downsampled full cloud 
    _to_save->points.assign(centroids.begin(), centroids.end());    
    _to_save->width = 1; 
    _to_save->height = _to_save->points.size(); // make sure again the format of the downsampled point cloud 
    ROS_INFO_STREAM("\033[1;32m Downsampled pointcloud have: " << _to_save->points.size() << " points.\033[0m");   
    cout << endl;
} // octreeDownsampling


void Removerter::makeGlobalMap( void )
{
    // transform local to global and merging the scans 
    map_global_orig_->clear();
    map_global_curr_->clear();

    mergeScansWithinGlobalCoord(scans_, scan_poses_, map_global_orig_);
    ROS_INFO_STREAM("\033[1;32m Map pointcloud (having redundant points) have: " << map_global_orig_->points.size() << " points.\033[0m");   
    ROS_INFO_STREAM("\033[1;32m Downsampling leaf size is " << kDownsampleVoxelSize << " m.\033[0m"); 

    // remove repeated (redundant) points
    // - using OctreePointCloudVoxelCentroid for downsampling 
    // - For a large-size point cloud should use OctreePointCloudVoxelCentroid rather VoxelGrid
    octreeDownsampling(map_global_orig_, map_global_curr_);

    // save the original cloud 
    if( kFlagSaveMapPointcloud ) {
        // in global coord
        std::string static_global_file_name = save_pcd_directory_ + "OriginalNoisyMapGlobal.pcd";
        pcl::io::savePCDFileBinary(static_global_file_name, *map_global_curr_);
        ROS_INFO_STREAM("\033[1;32m The original pointcloud is saved (global coord): " << static_global_file_name << "\033[0m");   

        // in local coord (i.e., base_node_idx == 0 means a start idx is the identity pose)
        int base_node_idx = base_node_idx_;    
        pcl::PointCloud<PointType>::Ptr map_local_curr (new pcl::PointCloud<PointType>);
        transformGlobalMapToLocal(map_global_curr_, base_node_idx, map_local_curr);
        std::string static_local_file_name = save_pcd_directory_ + "OriginalNoisyMapLocal.pcd";
        pcl::io::savePCDFileBinary(static_local_file_name, *map_local_curr);
        ROS_INFO_STREAM("\033[1;32m The original pointcloud is saved (local coord): " << static_local_file_name << "\033[0m");   
    }
    // make tree (for fast ball search for the projection to make a map range image later)
    // if(kUseSubsetMapCloud) // NOT recommend to use for under 5 million points map input
    //     kdtree_map_global_curr_->setInputCloud(map_global_curr_);

    // save current map into history
    // TODO
    // if(save_history_on_memory_)
    //     saveCurrentStaticMapHistory();

} // makeGlobalMap
 

void Removerter::transformGlobalMapSubsetToLocal(int _base_scan_idx)
{
    Eigen::Matrix4d base_pose_inverse = scan_inverse_poses_.at(_base_scan_idx);
    
    // global to local (global2local)
    map_local_curr_->clear();
    pcl::transformPointCloud(*map_subset_global_curr_, *map_local_curr_, base_pose_inverse);
    pcl::transformPointCloud(*map_local_curr_, *map_local_curr_, kSE3MatExtrinsicPoseBasetoLiDAR);

} // transformGlobalMapSubsetToLocal


void Removerter::transformGlobalMapToLocal(int _base_scan_idx)
{
    Eigen::Matrix4d base_pose_inverse = scan_inverse_poses_.at(_base_scan_idx);
    
    // global to local (global2local)
    map_local_curr_->clear();
    pcl::transformPointCloud(*map_global_curr_, *map_local_curr_, base_pose_inverse);
    pcl::transformPointCloud(*map_local_curr_, *map_local_curr_, kSE3MatExtrinsicPoseBasetoLiDAR);

} // transformGlobalMapToLocal


void Removerter::transformGlobalMapToLocal(int _base_scan_idx, pcl::PointCloud<PointType>::Ptr& _map_local)
{
    Eigen::Matrix4d base_pose_inverse = scan_inverse_poses_.at(_base_scan_idx);
    
    // global to local (global2local)
    _map_local->clear();
    pcl::transformPointCloud(*map_global_curr_, *_map_local, base_pose_inverse);
    pcl::transformPointCloud(*_map_local, *_map_local, kSE3MatExtrinsicPoseBasetoLiDAR);

} // transformGlobalMapToLocal


void Removerter::transformGlobalMapToLocal(
    const pcl::PointCloud<PointType>::Ptr& _map_global, 
    int _base_scan_idx, pcl::PointCloud<PointType>::Ptr& _map_local)
{
    Eigen::Matrix4d base_pose_inverse = scan_inverse_poses_.at(_base_scan_idx);
    
    // global to local (global2local)
    _map_local->clear();
    pcl::transformPointCloud(*_map_global, *_map_local, base_pose_inverse);
    pcl::transformPointCloud(*_map_local, *_map_local, kSE3MatExtrinsicPoseBasetoLiDAR);

} // transformGlobalMapToLocal


void Removerter::parseMapPointcloudSubsetUsingPtIdx( std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save ) 
{
    // extractor
    pcl::ExtractIndices<PointType> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
    extractor.setInputCloud(map_global_curr_); 
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

    // parse 
    _ptcloud_to_save->clear();
    extractor.filter(*_ptcloud_to_save);
} // parseMapPointcloudSubsetUsingPtIdx


void Removerter::parseStaticMapPointcloudUsingPtIdx( std::vector<int>& _point_indexes ) 
{
    // extractor
    pcl::ExtractIndices<PointType> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
    extractor.setInputCloud(map_global_curr_); 
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

    // parse 
    map_global_curr_static_->clear();
    extractor.filter(*map_global_curr_static_);
} // parseStaticMapPointcloudUsingPtIdx


void Removerter::parseDynamicMapPointcloudUsingPtIdx( std::vector<int>& _point_indexes )
{
    // extractor
    pcl::ExtractIndices<PointType> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
    extractor.setInputCloud(map_global_curr_); 
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

    // parse 
    map_global_curr_dynamic_->clear();
    extractor.filter(*map_global_curr_dynamic_);
} // parseDynamicMapPointcloudUsingPtIdx


void Removerter::saveCurrentStaticAndDynamicPointCloudGlobal( void )
{
    if( ! kFlagSaveMapPointcloud )
        return;

    std::string curr_res_alpha_str = std::to_string(curr_res_alpha_);

    // dynamic 
    std::string dyna_file_name = map_dynamic_save_dir_ + "/DynamicMapMapsideGlobalResX" + curr_res_alpha_str + ".pcd";
    pcl::io::savePCDFileBinary(dyna_file_name, *map_global_curr_dynamic_);
    ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << dyna_file_name << "\033[0m");   

    // static 
    std::string static_file_name = map_static_save_dir_ + "/StaticMapMapsideGlobalResX" + curr_res_alpha_str + ".pcd";
    pcl::io::savePCDFileBinary(static_file_name, *map_global_curr_static_);
    ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << static_file_name << "\033[0m");   
} // saveCurrentStaticAndDynamicPointCloudGlobal


void Removerter::saveCurrentStaticAndDynamicPointCloudLocal( int _base_node_idx )
{
    if( ! kFlagSaveMapPointcloud )
        return;

    std::string curr_res_alpha_str = std::to_string(curr_res_alpha_);

    // dynamic 
    pcl::PointCloud<PointType>::Ptr map_local_curr_dynamic (new pcl::PointCloud<PointType>);
    transformGlobalMapToLocal(map_global_curr_dynamic_, _base_node_idx, map_local_curr_dynamic);
    std::string dyna_file_name = map_dynamic_save_dir_ + "/DynamicMapMapsideLocalResX" + curr_res_alpha_str + ".pcd";
    pcl::io::savePCDFileBinary(dyna_file_name, *map_local_curr_dynamic);
    ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << dyna_file_name << "\033[0m");   

    // static 
    pcl::PointCloud<PointType>::Ptr map_local_curr_static (new pcl::PointCloud<PointType>);
    transformGlobalMapToLocal(map_global_curr_static_, _base_node_idx, map_local_curr_static);
    std::string static_file_name = map_static_save_dir_ + "/StaticMapMapsideLocalResX" + curr_res_alpha_str + ".pcd";
    pcl::io::savePCDFileBinary(static_file_name, *map_local_curr_static);
    ROS_INFO_STREAM("\033[1;32m -- a pointcloud is saved: " << static_file_name << "\033[0m");   

} // saveCurrentStaticAndDynamicPointCloudLocal


std::vector<int> Removerter::calcDescrepancyAndParseDynamicPointIdx
    (const cv::Mat& _scan_rimg, const cv::Mat& _diff_rimg, const cv::Mat& _map_rimg_ptidx)
{
    int num_dyna_points {0}; // TODO: tracking the number of dynamic-assigned points and decide when to stop removing (currently just fixed iteration e.g., [2.5, 2.0, 1.5])

    std::vector<int> dynamic_point_indexes;
    for (int row_idx = 0; row_idx < _diff_rimg.rows; row_idx++) {
        for (int col_idx = 0; col_idx < _diff_rimg.cols; col_idx++) {
            float this_diff = _diff_rimg.at<float>(row_idx, col_idx);
            float this_range = _scan_rimg.at<float>(row_idx, col_idx);

            float adaptive_coeff = 0.05; // meter, // i.e., if 4m apart point, it should be 0.4m be diff (nearer) wrt the query
            float adaptive_dynamic_descrepancy_threshold = adaptive_coeff * this_range; // adaptive descrepancy threshold 
            // float adaptive_dynamic_descrepancy_threshold = 0.1;

            if( this_diff < kValidDiffUpperBound // exclude no-point pixels either on scan img or map img (100 is roughly 100 meter)
                && this_diff > adaptive_dynamic_descrepancy_threshold /* dynamic */) 
            {  // dynamic
                int this_point_idx_in_global_map = _map_rimg_ptidx.at<int>(row_idx, col_idx);
                dynamic_point_indexes.emplace_back(this_point_idx_in_global_map);

                // num_dyna_points++; // TODO
            } 
        } 
    }

    return dynamic_point_indexes;
} // calcDescrepancyAndParseDynamicPointIdx


void Removerter::takeGlobalMapSubsetWithinBall( int _center_scan_idx )
{
    Eigen::Matrix4d center_pose_se3 = scan_poses_.at(_center_scan_idx);
    PointType center_pose;
    center_pose.x = float(center_pose_se3(0, 3));
    center_pose.y = float(center_pose_se3(1, 3));
    center_pose.z = float(center_pose_se3(2, 3));

    std::vector<int> subset_indexes;
    std::vector<float> pointSearchSqDisGlobalMap;
    kdtree_map_global_curr_->radiusSearch(center_pose, kBallSize, subset_indexes, pointSearchSqDisGlobalMap, 0);
    parseMapPointcloudSubsetUsingPtIdx(subset_indexes, map_subset_global_curr_);
} // takeMapSubsetWithinBall


std::vector<int> Removerter::calcDescrepancyAndParseDynamicPointIdxForEachScan( std::pair<int, int> _rimg_shape )
{   
    std::vector<int> dynamic_point_indexes;
    // dynamic_point_indexes.reserve(100000);
    for(std::size_t idx_scan=0; idx_scan < scans_.size(); ++idx_scan) {            
        // curr scan 
        pcl::PointCloud<PointType>::Ptr _scan = scans_.at(idx_scan);

        // scan's pointcloud to range img 
        cv::Mat scan_rimg = scan2RangeImg(_scan, kFOV, _rimg_shape); // openMP inside

        // map's pointcloud to range img 
        if( kUseSubsetMapCloud ) {
            takeGlobalMapSubsetWithinBall(idx_scan);
            transformGlobalMapSubsetToLocal(idx_scan); // the most time comsuming part 1 
        } else {
            // if the input map size (of a batch) is short, just using this line is more fast. 
            // - e.g., 100-1000m or ~5 million points are ok, empirically more than 10Hz 
            transformGlobalMapToLocal(idx_scan);
        }
        auto [map_rimg, map_rimg_ptidx] = map2RangeImg(map_local_curr_, kFOV, _rimg_shape); // the most time comsuming part 2 -> so openMP applied inside

        // diff range img 
        const int kNumRimgRow = _rimg_shape.first;
        const int kNumRimgCol = _rimg_shape.second;
        cv::Mat diff_rimg = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(0.0)); // float matrix, save range value 
        cv::absdiff(scan_rimg, map_rimg, diff_rimg);

        // parse dynamic points' indexes: rule: If a pixel value of diff_rimg is larger, scan is the further - means that pixel of submap is likely dynamic.
        std::vector<int> this_scan_dynamic_point_indexes = calcDescrepancyAndParseDynamicPointIdx(scan_rimg, diff_rimg, map_rimg_ptidx);
        dynamic_point_indexes.insert(dynamic_point_indexes.end(), this_scan_dynamic_point_indexes.begin(), this_scan_dynamic_point_indexes.end());

        // visualization 
        pubRangeImg(scan_rimg, scan_rimg_msg_, scan_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(map_rimg, map_rimg_msg_, map_rimg_msg_publisher_, kRangeColorAxis);
        pubRangeImg(diff_rimg, diff_rimg_msg_, diff_rimg_msg_publisher_, kRangeColorAxisForDiff);

        std::pair<float, float> kRangeColorAxisForPtIdx {0.0, float(map_global_curr_->points.size())};
        pubRangeImg(map_rimg_ptidx, map_rimg_ptidx_msg_, map_rimg_ptidx_msg_publisher_, kRangeColorAxisForPtIdx);

        publishPointcloud2FromPCLptr(scan_publisher_, _scan);
    } // for_each scan Done

    // remove repeated indexes
    std::set<int> dynamic_point_indexes_set (dynamic_point_indexes.begin(), dynamic_point_indexes.end());
    std::vector<int> dynamic_point_indexes_unique (dynamic_point_indexes_set.begin(), dynamic_point_indexes_set.end());

    return dynamic_point_indexes_unique;
} // calcDescrepancyForEachScan


std::vector<int> Removerter::getStaticIdxFromDynamicIdx(const std::vector<int>& _dynamic_point_indexes, int _num_all_points)
{
    std::vector<int> pt_idx_all = linspace<int>(0, _num_all_points, _num_all_points);

    std::set<int> pt_idx_all_set (pt_idx_all.begin(), pt_idx_all.end());
    for(auto& _dyna_pt_idx: _dynamic_point_indexes) {
        pt_idx_all_set.erase(_dyna_pt_idx);
    }

    std::vector<int> static_point_indexes (pt_idx_all_set.begin(), pt_idx_all_set.end());
    return static_point_indexes;
} // getStaticIdxFromDynamicIdx


std::vector<int> Removerter::getGlobalMapStaticIdxFromDynamicIdx(const std::vector<int>& _dynamic_point_indexes)
{
    int num_all_points = map_global_curr_->points.size();
    return getStaticIdxFromDynamicIdx(_dynamic_point_indexes, num_all_points);
} // getGlobalMapStaticIdxFromDynamicIdx



void Removerter::saveCurrentStaticMapHistory(void)
{
    // deep copy
    pcl::PointCloud<PointType>::Ptr map_global_curr_static (new pcl::PointCloud<PointType>);
    *map_global_curr_static = *map_global_curr_;  

    // save 
    static_map_global_history_.emplace_back(map_global_curr_static);
} // saveCurrentStaticMapHistory


void Removerter::removeOnce( float _res_alpha )
{
    // filter spec (i.e., a shape of the range image) 
    curr_res_alpha_ = _res_alpha;

    std::pair<int, int> rimg_shape = resetRimgSize(kFOV, _res_alpha);
    float deg_per_pixel = 1.0 / _res_alpha;
    ROS_INFO_STREAM("\033[1;32m Removing starts with resolution: x" << _res_alpha << " (" << deg_per_pixel << " deg/pixel)\033[0m");   
    ROS_INFO_STREAM("\033[1;32m -- The range image size is: [" << rimg_shape.first << ", " << rimg_shape.second << "].\033[0m");   
    ROS_INFO_STREAM("\033[1;32m -- The number of map points: " << map_global_curr_->points.size() << "\033[0m");  
    ROS_INFO_STREAM("\033[1;32m -- ... starts cleaning ... " << "\033[0m");  

    // map-side removal: remove and get dynamic (will be removed) points' index set
    std::vector<int> dynamic_point_indexes = calcDescrepancyAndParseDynamicPointIdxForEachScan( rimg_shape );
    ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points: " << dynamic_point_indexes.size() << "\033[0m"); 
    parseDynamicMapPointcloudUsingPtIdx(dynamic_point_indexes);  

    // static_point_indexes == complemently indexing dynamic_point_indexes
    std::vector<int> static_point_indexes = getGlobalMapStaticIdxFromDynamicIdx(dynamic_point_indexes); 
    ROS_INFO_STREAM("\033[1;32m -- The number of static points: " << static_point_indexes.size() << "\033[0m");  
    parseStaticMapPointcloudUsingPtIdx(static_point_indexes);

    // Update the current map and reset the tree
    map_global_curr_->clear();
    *map_global_curr_ = *map_global_curr_static_;

    // if(kUseSubsetMapCloud) // NOT recommend to use for under 5 million points map input
    //     kdtree_map_global_curr_->setInputCloud(map_global_curr_); 

} // removeOnce


void Removerter::revertOnce( float _res_alpha )
{
    std::pair<int, int> rimg_shape = resetRimgSize(kFOV, _res_alpha);
    float deg_per_pixel = 1.0 / _res_alpha;    
    ROS_INFO_STREAM("\033[1;32m Reverting starts with resolution: x" << _res_alpha << " (" << deg_per_pixel << " deg/pixel)\033[0m");   
    ROS_INFO_STREAM("\033[1;32m -- The range image size is: [" << rimg_shape.first << ", " << rimg_shape.second << "].\033[0m");   
    ROS_INFO_STREAM("\033[1;32m -- ... TODO ... \033[0m");   

    // TODO

} // revertOnce


void Removerter::parsePointcloudSubsetUsingPtIdx( const pcl::PointCloud<PointType>::Ptr& _ptcloud_orig,
            std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save ) 
{
    // extractor
    pcl::ExtractIndices<PointType> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(_point_indexes);
    extractor.setInputCloud(_ptcloud_orig); 
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

    // parse 
    _ptcloud_to_save->clear();
    extractor.filter(*_ptcloud_to_save);
} // parsePointcloudSubsetUsingPtIdx


pcl::PointCloud<PointType>::Ptr Removerter::local2global(const pcl::PointCloud<PointType>::Ptr& _scan_local, int _scan_idx)
{
    Eigen::Matrix4d scan_pose = scan_poses_.at(_scan_idx);

    pcl::PointCloud<PointType>::Ptr scan_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*_scan_local, *scan_global, kSE3MatExtrinsicLiDARtoPoseBase);
    pcl::transformPointCloud(*scan_global, *scan_global, scan_pose);

    return scan_global;
}

pcl::PointCloud<PointType>::Ptr Removerter::global2local(const pcl::PointCloud<PointType>::Ptr& _scan_global, int _scan_idx)
{
    Eigen::Matrix4d base_pose_inverse = scan_inverse_poses_.at(_scan_idx);
    
    pcl::PointCloud<PointType>::Ptr scan_local(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*_scan_global, *scan_local, base_pose_inverse);
    pcl::transformPointCloud(*scan_local, *scan_local, kSE3MatExtrinsicPoseBasetoLiDAR);

    return scan_local;
}

std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> 
    Removerter::removeDynamicPointsOfScanByKnn ( int _scan_idx )
{    
    // curr scan (in local coord)
    pcl::PointCloud<PointType>::Ptr scan_orig = scans_.at(_scan_idx); 
    auto scan_pose = scan_poses_.at(_scan_idx);

    // curr scan (in global coord)
    pcl::PointCloud<PointType>::Ptr scan_orig_global = local2global(scan_orig, _scan_idx);
    kdtree_scan_global_curr_->setInputCloud(scan_orig_global); 
    int num_points_of_a_scan = scan_orig_global->points.size();

    // 
    pcl::PointCloud<PointType>::Ptr scan_static_global (new pcl::PointCloud<PointType>); 
    pcl::PointCloud<PointType>::Ptr scan_dynamic_global (new pcl::PointCloud<PointType>); 
    for (std::size_t pt_idx = 0; pt_idx < num_points_of_a_scan; pt_idx++)
    {
        std::vector<int> topk_indexes_scan;
        std::vector<float> topk_L2dists_scan;
        kdtree_scan_global_curr_->nearestKSearch(scan_orig_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_scan, topk_L2dists_scan);
        float sum_topknn_dists_in_scan = accumulate( topk_L2dists_scan.begin(), topk_L2dists_scan.end(), 0.0);
        float avg_topknn_dists_in_scan = sum_topknn_dists_in_scan / float(kNumKnnPointsToCompare);

        std::vector<int> topk_indexes_map;
        std::vector<float> topk_L2dists_map;
        kdtree_map_global_curr_->nearestKSearch(scan_orig_global->points[pt_idx], kNumKnnPointsToCompare, topk_indexes_map, topk_L2dists_map);
        float sum_topknn_dists_in_map = accumulate( topk_L2dists_map.begin(), topk_L2dists_map.end(), 0.0);
        float avg_topknn_dists_in_map = sum_topknn_dists_in_map / float(kNumKnnPointsToCompare);

        // 
        if ( std::abs(avg_topknn_dists_in_scan - avg_topknn_dists_in_map) < kScanKnnAndMapKnnAvgDiffThreshold) {
            scan_static_global->push_back(scan_orig_global->points[pt_idx]);
        } else {
            scan_dynamic_global->push_back(scan_orig_global->points[pt_idx]);
        }
    }

    // again global2local because later in the merging global map function, which requires scans within each local coord. 
    pcl::PointCloud<PointType>::Ptr scan_static_local = global2local(scan_static_global, _scan_idx);
    pcl::PointCloud<PointType>::Ptr scan_dynamic_local = global2local(scan_dynamic_global, _scan_idx);

    ROS_INFO_STREAM("\033[1;32m The scan " << sequence_valid_scan_paths_.at(_scan_idx) << "\033[0m"); 
    ROS_INFO_STREAM("\033[1;32m -- The number of static points in a scan: " << scan_static_local->points.size() << "\033[0m"); 
    ROS_INFO_STREAM("\033[1;32m -- The number of dynamic points in a scan: " << num_points_of_a_scan - scan_static_local->points.size() << "\033[0m"); 

    publishPointcloud2FromPCLptr(static_curr_scan_publisher_, scan_static_local);
    publishPointcloud2FromPCLptr(dynamic_curr_scan_publisher_, scan_dynamic_local);
    usleep( kPauseTimeForClearStaticScanVisualization );

    return std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> (scan_static_local, scan_dynamic_local);

} // removeDynamicPointsOfScanByKnn


void Removerter::saveStaticScan( int _scan_idx, const pcl::PointCloud<PointType>::Ptr& _ptcloud )
{

    std::string file_name_orig = sequence_valid_scan_names_.at(_scan_idx);
    std::string file_name = scan_static_save_dir_ + "/" + file_name_orig + ".pcd";
    ROS_INFO_STREAM("\033[1;32m Scan " << _scan_idx << "'s static points is saved (" << file_name << ")\033[0m");   
    pcl::io::savePCDFileBinary(file_name, *_ptcloud);
} // saveStaticScan


void Removerter::saveDynamicScan( int _scan_idx, const pcl::PointCloud<PointType>::Ptr& _ptcloud )
{
    std::string file_name_orig = sequence_valid_scan_names_.at(_scan_idx);
    std::string file_name = scan_dynamic_save_dir_ + "/" + file_name_orig + ".pcd";
    ROS_INFO_STREAM("\033[1;32m Scan " << _scan_idx << "'s static points is saved (" << file_name << ")\033[0m");   
    pcl::io::savePCDFileBinary(file_name, *_ptcloud);
} // saveDynamicScan


void Removerter::saveCleanedScans(void)
{
    if( ! kFlagSaveCleanScans )
        return;

    for(std::size_t idx_scan=0; idx_scan < scans_static_.size(); idx_scan++) {  
        saveStaticScan(idx_scan, scans_static_.at(idx_scan));
        saveDynamicScan(idx_scan, scans_dynamic_.at(idx_scan));
    }
} // saveCleanedScans


void Removerter::saveMapPointcloudByMergingCleanedScans(void)
{
    // merge for verification
    if( ! kFlagSaveMapPointcloud ) 
        return;

    // static map
    {
        pcl::PointCloud<PointType>::Ptr map_global_static_scans_merged_to_verify_full (new pcl::PointCloud<PointType>); 
        pcl::PointCloud<PointType>::Ptr map_global_static_scans_merged_to_verify (new pcl::PointCloud<PointType>); 
        mergeScansWithinGlobalCoord(scans_static_, scan_poses_, map_global_static_scans_merged_to_verify_full);
        octreeDownsampling(map_global_static_scans_merged_to_verify_full, map_global_static_scans_merged_to_verify);

        // global
        std::string local_file_name = map_static_save_dir_ + "/StaticMapScansideMapGlobal.pcd";
        pcl::io::savePCDFileBinary(local_file_name, *map_global_static_scans_merged_to_verify);
        ROS_INFO_STREAM("\033[1;32m  [For verification] A static pointcloud (cleaned scans merged) is saved (global coord): " << local_file_name << "\033[0m");   

        // local 
        pcl::PointCloud<PointType>::Ptr map_local_static_scans_merged_to_verify (new pcl::PointCloud<PointType>);
        int base_node_idx = base_node_idx_;
        transformGlobalMapToLocal(map_global_static_scans_merged_to_verify, base_node_idx, map_local_static_scans_merged_to_verify);
        std::string global_file_name = map_static_save_dir_ + "/StaticMapScansideMapLocal.pcd";
        pcl::io::savePCDFileBinary(global_file_name, *map_local_static_scans_merged_to_verify);
        ROS_INFO_STREAM("\033[1;32m  [For verification] A static pointcloud (cleaned scans merged) is saved (local coord): " << global_file_name << "\033[0m");  
    } 

    // dynamic map
    {
        pcl::PointCloud<PointType>::Ptr map_global_dynamic_scans_merged_to_verify_full (new pcl::PointCloud<PointType>); 
        pcl::PointCloud<PointType>::Ptr map_global_dynamic_scans_merged_to_verify (new pcl::PointCloud<PointType>); 
        mergeScansWithinGlobalCoord(scans_dynamic_, scan_poses_, map_global_dynamic_scans_merged_to_verify_full);
        octreeDownsampling(map_global_dynamic_scans_merged_to_verify_full, map_global_dynamic_scans_merged_to_verify);

        // global
        std::string local_file_name = map_dynamic_save_dir_ + "/DynamicMapScansideMapGlobal.pcd";
        pcl::io::savePCDFileBinary(local_file_name, *map_global_dynamic_scans_merged_to_verify);
        ROS_INFO_STREAM("\033[1;32m  [For verification] A dynamic pointcloud (cleaned scans merged) is saved (global coord): " << local_file_name << "\033[0m");   

        // local 
        pcl::PointCloud<PointType>::Ptr map_local_dynamic_scans_merged_to_verify (new pcl::PointCloud<PointType>);
        int base_node_idx = base_node_idx_;
        transformGlobalMapToLocal(map_global_dynamic_scans_merged_to_verify, base_node_idx, map_local_dynamic_scans_merged_to_verify);
        std::string global_file_name = map_dynamic_save_dir_ + "/DynamicMapScansideMapLocal.pcd";
        pcl::io::savePCDFileBinary(global_file_name, *map_local_dynamic_scans_merged_to_verify);
        ROS_INFO_STREAM("\033[1;32m  [For verification] A dynamic pointcloud (cleaned scans merged) is saved (local coord): " << global_file_name << "\033[0m");  
    } 
} // saveMapPointcloudByMergingCleanedScans


void Removerter::scansideRemovalForEachScan( void )
{
    // for fast scan-side neighbor search 
    kdtree_map_global_curr_->setInputCloud(map_global_curr_); 

    // for each scan
    for(std::size_t idx_scan=0; idx_scan < scans_.size(); idx_scan++) {
        auto [this_scan_static, this_scan_dynamic] = removeDynamicPointsOfScanByKnn(idx_scan);
        scans_static_.emplace_back(this_scan_static);
        scans_dynamic_.emplace_back(this_scan_dynamic);
    }
} // scansideRemovalForEachScan


void Removerter::scansideRemovalForEachScanAndSaveThem( void )
{
    scansideRemovalForEachScan();
    saveCleanedScans();
    saveMapPointcloudByMergingCleanedScans();
} // scansideRemovalForEachScanAndSaveThem


void Removerter::run( void )
{
    // load scan and poses
    parseValidScanInfo();
    readValidScans();

    // construct initial map using the scans and the corresponding poses 
    makeGlobalMap();

    // map-side removals
    for(float _rm_res: remove_resolution_list_) {
        removeOnce( _rm_res );
    } 

    // if you want to every iteration's map data, place below two lines to inside of the above for loop 
    saveCurrentStaticAndDynamicPointCloudGlobal(); // if you want to save within the global points uncomment this line
    saveCurrentStaticAndDynamicPointCloudLocal(base_node_idx_); // w.r.t specific node's coord. 0 means w.r.t the start node, as an Identity.

    // TODO
    // map-side reverts
    // if you want to remove as much as possible, you can use omit this steps
    for(float _rv_res: revert_resolution_list_) {
        revertOnce( _rv_res );
    }

    // scan-side removals
    scansideRemovalForEachScanAndSaveThem();

}