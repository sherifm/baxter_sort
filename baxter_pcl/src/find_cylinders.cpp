#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <baxter_pcl/pcl_cylinder.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
ros::Publisher pub_obj;
ros::Publisher pub1;
ros::Publisher pub1_obj;
typedef pcl::PointXYZ PointT;

float deg2rad(float alpha)
{
  return (alpha * 0.017453293f);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Do data processing here...
  // run ransac to find floor
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
 pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg1; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients_cylinder1 (new pcl::ModelCoefficients), coefficients_cylinder2 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  pcl::PointIndices::Ptr inliers_cylinder1 (new pcl::PointIndices), inliers_cylinder2 (new pcl::PointIndices);
 std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
pcl::fromROSMsg(*input, *cloud);
  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.5, 1); //data points further away than 1 meter are filtered
  pass.filter (*cloud_filtered);

  //x filter
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.25, 0.25); //data points further away than 1 meter are filtered
  pass.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg1.setOptimizeCoefficients (true);
  seg1.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg1.setNormalDistanceWeight (0.1);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setMaxIterations (100);
  seg1.setDistanceThreshold (0.03);
  seg1.setInputCloud (cloud_filtered);
  seg1.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg1.segment (*inliers_plane, *coefficients_plane);
  //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);
float radius=0;
 float height=0;
 int i;
pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());

 for(i=0;i<10;i++){
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);  
  extract.filter (*cloud_cylinder);
height=height+cloud_cylinder->points[cloud_cylinder->width-1].y - cloud_cylinder->points[0].y;
radius=radius+coefficients_cylinder->values[6];
 }

 radius=radius/10;
 height=height/10;
geometry_msgs::PointStamped pt;
baxter_pcl::pcl_cylinder pcl;
pub.publish(*cloud_cylinder);
pcl.header.seq = cloud_cylinder->header.seq;
pcl.cylinder = 1;
pcl.header.frame_id = cloud_cylinder->header.frame_id;
pcl.point.x = cloud_cylinder->points[cloud_cylinder->width-1].x;
pcl.point.y = cloud_cylinder->points[cloud_cylinder->width-1].y+height/2;
pcl.point.z = cloud_cylinder->points[cloud_cylinder->width-1].z;
 pcl.radius=radius;
 pcl.height=height;
pub1.publish(pcl);

 pcl::PointCloud<PointT>::Ptr cloud_cylinder1 (new pcl::PointCloud<PointT> ());
extract.setNegative (true);
extract.filter (*cloud_filtered2);
extract_normals.setNegative (true);
extract_normals.setInputCloud (cloud_normals2);
extract_normals.setIndices (inliers_cylinder);
extract_normals.filter (*cloud_normals2);
seg.setInputCloud (cloud_filtered2);
seg.setInputNormals (cloud_normals2);

for(i=0;i<10;i++){
  seg.segment (*inliers_cylinder1, *coefficients_cylinder1);
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder1);
  extract.setNegative (false);  
  extract.filter (*cloud_cylinder1);
height=height+cloud_cylinder1->points[cloud_cylinder1->width-1].y - cloud_cylinder1->points[0].y;
radius=radius+coefficients_cylinder1->values[6];
 }
 radius=radius/10;
 height=height/10;

pub_obj.publish(*cloud_cylinder1);
pcl.header.seq = cloud_cylinder->header.seq;
 pcl.cylinder = 2;
//pt.header.stamp = 0;
pcl.header.frame_id = cloud_cylinder1->header.frame_id;
pcl.point.x = cloud_cylinder1->points[cloud_cylinder1->width-1].x;
pcl.point.y = cloud_cylinder1->points[cloud_cylinder1->width-1].y+height/2;
pcl.point.z = cloud_cylinder1->points[cloud_cylinder1->width-1].z;

 pcl.radius=radius;
 pcl.height=height;
pub1.publish(pcl);

 pcl::PointCloud<PointT>::Ptr cloud_cylinder2 (new pcl::PointCloud<PointT> ());
extract.setNegative (true);
extract.filter (*cloud_filtered2);
extract_normals.setNegative (true);
extract_normals.setInputCloud (cloud_normals2);
extract_normals.setIndices (inliers_cylinder1);
extract_normals.filter (*cloud_normals2);
seg.setInputCloud (cloud_filtered2);
seg.setInputNormals (cloud_normals2);

for(i=0;i<10;i++){
  seg.segment (*inliers_cylinder2, *coefficients_cylinder2);
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder2);
  extract.setNegative (false);  
  extract.filter (*cloud_cylinder2);
height=height+cloud_cylinder2->points[cloud_cylinder2->width-1].y - cloud_cylinder2->points[0].y;
radius=radius+coefficients_cylinder2->values[6];
 }
 radius=radius/10;
 height=height/10;

pub1_obj.publish(*cloud_cylinder2);
pcl.header.seq = cloud_cylinder->header.seq;
 pcl.cylinder = 3;
//pt.header.stamp = 0;
pcl.header.frame_id = cloud_cylinder2->header.frame_id;
pcl.point.x = cloud_cylinder2->points[cloud_cylinder2->width-1].x;
pcl.point.y = cloud_cylinder2->points[cloud_cylinder2->width-1].y+height/2;
pcl.point.z = cloud_cylinder2->points[cloud_cylinder2->width-1].z;
 pcl.radius=radius;
 pcl.height=height;
pub1.publish(pcl);


}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "find_cylinders");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cylinder1", 1);
  pub_obj = nh.advertise<sensor_msgs::PointCloud2> ("cylinder2", 1);
 pub1_obj = nh.advertise<sensor_msgs::PointCloud2> ("cylinder3", 1);
 pub1 = nh.advertise<baxter_pcl::pcl_cylinder> ("data", 1);

  // Spin
  ros::spin ();
}

