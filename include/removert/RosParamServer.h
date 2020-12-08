#pragma once

#include "removert/utility.h"

class RosNodeHandle
{
public:
    ros::NodeHandle nh_super;
}; // class: RosNodeHandle


class RosParamServer: public RosNodeHandle
{
public:
    // 
    ros::NodeHandle & nh;

    //
    std::string pointcloud_topic;

    // removert params 
    float kVFOV;
    float kHFOV;
    std::pair<float, float> kFOV;

    // sequence info 
    std::vector<double> kVecExtrinsicLiDARtoPoseBase; 
    Eigen::Matrix4d kSE3MatExtrinsicLiDARtoPoseBase;
        // Base is where of the pose writtened (e.g., for KITTI, poses is usually in camera)
        // if the pose file is obtained via lidar odometry itself, then kMatExtrinsicLiDARtoBase is eye(4)
    Eigen::Matrix4d kSE3MatExtrinsicPoseBasetoLiDAR;

    // sequence bin files
    bool isScanFileKITTIFormat_;

    std::string sequence_scan_dir_;
    std::vector<std::string> sequence_scan_names_;
    std::vector<std::string> sequence_scan_paths_;
    int num_total_scans_of_sequence_;
    float kDownsampleVoxelSize;

    // sequence pose file
    std::string sequence_pose_path_;
    std::vector<Eigen::Matrix4d> sequence_scan_poses_;
    std::vector<Eigen::Matrix4d> sequence_scan_inverse_poses_; // used for global to local

    // target region to removerting 
    int start_idx_;
    int end_idx_;

    bool use_keyframe_gap_; 
    bool use_keyframe_meter_; 
    int keyframe_gap_;
    float keyframe_gap_meter_;

    // 
    std::vector<float> remove_resolution_list_;
    std::vector<float> revert_resolution_list_;

    // 
    int kNumOmpCores;

    //
    pcl::PointCloud<PointType>::Ptr single_scan;
    pcl::PointCloud<PointType>::Ptr projected_scan;

    // ros pub
    ros::Publisher scan_publisher_;
    ros::Publisher global_scan_publisher_;

    ros::Publisher original_map_local_publisher_;

    ros::Publisher curr_map_local_publisher_;
    ros::Publisher static_map_local_publisher_;
    ros::Publisher dynamic_map_local_publisher_;

    ros::Publisher static_curr_scan_publisher_;
    ros::Publisher dynamic_curr_scan_publisher_;

    float rimg_color_min_;
    float rimg_color_max_;
    std::pair<float, float> kRangeColorAxis; // meter
    std::pair<float, float> kRangeColorAxisForDiff; // meter 

    image_transport::ImageTransport ROSimg_transporter_;

    sensor_msgs::ImagePtr scan_rimg_msg_;
    image_transport::Publisher scan_rimg_msg_publisher_;

    sensor_msgs::ImagePtr map_rimg_msg_;
    image_transport::Publisher map_rimg_msg_publisher_;

    sensor_msgs::ImagePtr diff_rimg_msg_;
    image_transport::Publisher diff_rimg_msg_publisher_;

    sensor_msgs::ImagePtr map_rimg_ptidx_msg_;
    image_transport::Publisher map_rimg_ptidx_msg_publisher_;

    //
    bool kFlagSaveMapPointcloud;
    bool kFlagSaveCleanScans;
    std::string save_pcd_directory_;

public:
    RosParamServer();

}; // class: RosParamServer
