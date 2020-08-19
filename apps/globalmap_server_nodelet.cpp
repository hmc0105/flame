#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/io/pcd_io.h>

namespace flame1{
class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    // publish globalmap with "latched" publisher
    //floor_elimination();

    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    globalmap_pub.publish(globalmap);

  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;

  }
  void floor_elimination(){
    std::cout<<"Loaded:"<< globalmap->width*globalmap->height<<std::endl;

    pcl::PointCloud<PointT>::Ptr inlierPoints (new pcl::PointCloud<PointT>),
                                  inlierPoints_neg (new pcl::PointCloud<PointT>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(globalmap);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.segment(*inliers,*coefficients);
    pcl::copyPointCloud<PointT>(*globalmap,*inliers,*inlierPoints);
    pcl::io::savePCDFile<PointT>("/SACsegmentation_result.pcd",*inlierPoints);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(globalmap);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*inlierPoints_neg);
    pcl::io::savePCDFile<PointT>("/SACSEG_NEG.pcd",*inlierPoints_neg);
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;

  pcl::PointCloud<PointT>::Ptr globalmap;
};
}

PLUGINLIB_EXPORT_CLASS(flame1::GlobalmapServerNodelet, nodelet::Nodelet);

