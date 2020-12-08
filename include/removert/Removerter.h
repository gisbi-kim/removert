#pragma once

#include "removert/RosParamServer.h"

class Removerter : public RosParamServer
{
private:
    
    ros::Subscriber subLaserCloud; // TODO, for real-time integration (later)

    const float kFlagNoPOINT = 10000.0; // no point constant, 10000 has no meaning, but must be larger than the maximum scan range (e.g., 200 meters)
    const float kValidDiffUpperBound = 200.0; // must smaller than kFlagNoPOINT

    // Static sensitivity
    int kNumKnnPointsToCompare;// static sensitivity (increase this value, less static structure will be removed at the scan-side removal stage)
    float kScanKnnAndMapKnnAvgDiffThreshold; // static sensitivity (decrease this value, less static structure will be removed at the scan-side removal stage)

    std::vector<std::string> sequence_valid_scan_names_;
    std::vector<std::string> sequence_valid_scan_paths_;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_static_;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_dynamic_;

    std::string scan_static_save_dir_;
    std::string scan_dynamic_save_dir_;
    std::string map_static_save_dir_;
    std::string map_dynamic_save_dir_;
    
    std::vector<Eigen::Matrix4d> scan_poses_;
    std::vector<Eigen::Matrix4d> scan_inverse_poses_;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_global_curr_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_scan_global_curr_;

    pcl::PointCloud<PointType>::Ptr map_global_orig_;

    pcl::PointCloud<PointType>::Ptr map_global_curr_; // the M_i. i.e., removert is: M1 -> S1 + D1, D1 -> M2 , M2 -> S2 + D2 ... repeat ... 
    pcl::PointCloud<PointType>::Ptr map_local_curr_;

    pcl::PointCloud<PointType>::Ptr map_subset_global_curr_;

    pcl::PointCloud<PointType>::Ptr map_global_curr_static_; // the S_i
    pcl::PointCloud<PointType>::Ptr map_global_curr_dynamic_;  // the D_i

    pcl::PointCloud<PointType>::Ptr map_global_accumulated_static_; // TODO, the S_i after reverted
    pcl::PointCloud<PointType>::Ptr map_global_accumulated_dynamic_;  // TODO, the D_i after reverted 

    std::vector<pcl::PointCloud<PointType>::Ptr> static_map_global_history_; // TODO
    std::vector<pcl::PointCloud<PointType>::Ptr> dynamic_map_global_history_; // TODO

    float curr_res_alpha_; // just for tracking current status

    const int base_node_idx_ = 0;

    unsigned long kPauseTimeForClearStaticScanVisualization = 1000; // microsec

    // NOT recommend to use for under 5 million points map input (becausing not-using is just faster)
    const bool kUseSubsetMapCloud = false; 
    const float kBallSize = 80.0; // meter

public:
    Removerter();
    ~Removerter();

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg); // TODO (if removert run online)

    void allocateMemory();

    void parseValidScanInfo();
    void readValidScans();

    void mergeScansWithinGlobalCoord( 
            const std::vector<pcl::PointCloud<PointType>::Ptr>& _scans, 
            const std::vector<Eigen::Matrix4d>& _scans_poses,
            pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save );
    void octreeDownsampling(const pcl::PointCloud<PointType>::Ptr& _src, pcl::PointCloud<PointType>::Ptr& _to_save);
    void makeGlobalMap();

    void run(void);

    void removeOnce(float _res);
    void revertOnce(float _res);

    void saveCurrentStaticMapHistory(void); // the 0th element is a noisy (original input) (actually not static) map.
    void saveCurrentDynamicMapHistory(void);

    void takeGlobalMapSubsetWithinBall( int _center_scan_idx );
    void transformGlobalMapSubsetToLocal(int _base_scan_idx);

    void transformGlobalMapToLocal(int _base_scan_idx);
    void transformGlobalMapToLocal(int _base_scan_idx, pcl::PointCloud<PointType>::Ptr& _map_local);
    void transformGlobalMapToLocal(const pcl::PointCloud<PointType>::Ptr& _map_global, int _base_scan_idx, pcl::PointCloud<PointType>::Ptr& _map_local);

    cv::Mat scan2RangeImg(const pcl::PointCloud<PointType>::Ptr& _scan, 
                        const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                        const std::pair<int, int> _rimg_size);
    std::pair<cv::Mat, cv::Mat> map2RangeImg(const pcl::PointCloud<PointType>::Ptr& _scan, 
                        const std::pair<float, float> _fov, /* e.g., [vfov = 50 (upper 25, lower 25), hfov = 360] */
                        const std::pair<int, int> _rimg_size);

    std::vector<int> calcDescrepancyAndParseDynamicPointIdx (const cv::Mat& _scan_rimg, const cv::Mat& _diff_rimg, const cv::Mat& _map_rimg_ptidx);
    std::vector<int> calcDescrepancyAndParseDynamicPointIdxForEachScan( std::pair<int, int> _rimg_shape );

    std::vector<int> getStaticIdxFromDynamicIdx(const std::vector<int>& _dynamic_point_indexes, int _num_all_points);
    std::vector<int> getGlobalMapStaticIdxFromDynamicIdx(const std::vector<int>& _dynamic_point_indexes);

    void parsePointcloudSubsetUsingPtIdx( const pcl::PointCloud<PointType>::Ptr& _ptcloud_orig,
            std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save );
    void parseMapPointcloudSubsetUsingPtIdx( std::vector<int>& _point_indexes, pcl::PointCloud<PointType>::Ptr& _ptcloud_to_save );
    void parseStaticMapPointcloudUsingPtIdx( std::vector<int>& _point_indexes );
    void parseDynamicMapPointcloudUsingPtIdx( std::vector<int>& _point_indexes );

    void saveCurrentStaticAndDynamicPointCloudGlobal( void );
    void saveCurrentStaticAndDynamicPointCloudLocal( int _base_pose_idx  = 0);

    // void local2global(const pcl::PointCloud<PointType>::Ptr& _ptcloud_global, pcl::PointCloud<PointType>::Ptr& _ptcloud_local_to_save );
    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& _scan_local, int _scan_idx);
    pcl::PointCloud<PointType>::Ptr global2local(const pcl::PointCloud<PointType>::Ptr& _scan_global, int _scan_idx);

    // scan-side removal
    std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> removeDynamicPointsOfScanByKnn ( int _scan_idx );
    void removeDynamicPointsAndSaveStaticScanForEachScan( void );

    void scansideRemovalForEachScan(void);
    void saveCleanedScans(void);
    void saveMapPointcloudByMergingCleanedScans(void);
    void scansideRemovalForEachScanAndSaveThem(void);

    void saveStaticScan( int _scan_idx, const pcl::PointCloud<PointType>::Ptr& _ptcloud );
    void saveDynamicScan( int _scan_idx, const pcl::PointCloud<PointType>::Ptr& _ptcloud );

}; // Removerter