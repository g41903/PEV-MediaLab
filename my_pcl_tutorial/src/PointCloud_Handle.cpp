#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/foreach.hpp>

#define M_PI       3.14159265358979323846  /* pi */

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

Eigen::Matrix4f transform_1;
ros::Publisher pub;
double shift=0;

void setMetrix(float theta){
  //  |  sin   0  0   0 |
  //  |   0    1  0   0 |
  //  |  cos   0  0   0 |
  //  |   0    0  0   1 |
  transform_1 (0,0) = sin (theta);
  transform_1 (1,1) = 1;
  transform_1 (2,0) = cos (theta);
  transform_1 (3,3) = 1;
  //    (row, column)
}
void shiftMetrix(){
  transform_1 (0,3) = shift;
  shift = shift + 0.01;
}

void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  //do stuff with temp_cloud here
  PointCloud transformed_cloud;
  shiftMetrix();
  pcl::transformPointCloud (*temp_cloud, transformed_cloud, transform_1);
  pub.publish (transformed_cloud);
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  
  // ROS_INFO ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointCloud_handle");
  transform_1 = Eigen::Matrix4f::Zero();
  setMetrix(3*M_PI/4);
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("PointCloud", 1, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("PointCloud_out", 1);
  ros::spin();
}