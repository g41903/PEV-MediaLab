#include <cstdio>
#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include "sensor_msgs/PointCloud.h"
/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans>("assemble_scans");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(1,0), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void timerCallback(const ros::TimerEvent& e)
  {

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    AssembleScans srv;
    srv.request.begin = e.last_real;
    srv.request.end   = e.current_real;
    double begin = srv.request.begin.toSec();
    double end = srv.request.end.toSec();
    double dt = end - begin;
    // Make the service call
    if (client_.call(srv))
    {
    	ROS_INFO("Time %f ~ %f", begin, end	);
		//ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
    	ROS_INFO("dt = %f sec\nPublished Cloud with %u points", dt, srv.response.cloud.points.size()) ;
      pub_.publish(srv.response.cloud);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
	// ros::init(argc, argv, "periodic_snapshotter");
	// ros::NodeHandle n;
	// ROS_INFO("Waiting for [build_cloud] to be advertised");
	// ros::service::waitForService("build_cloud");
	// ROS_INFO("Found build_cloud! Starting the snapshotter");
	// PeriodicSnapshotter snapshotter;
	// ros::spin();
	// return 0;

	ros::init(argc, argv, "test_client");
	ros::NodeHandle n;
	ros::service::waitForService("assemble_scans2");
	ros::ServiceClient client = n.serviceClient<AssembleScans>("assemble_scans");
	AssembleScans srv;
	srv.request.begin = ros::Time(0,0);
	ros::Duration(5).sleep();
	srv.request.end   = ros::Time::now();
	if (client.call(srv))
	  printf("Got cloud with %u points\n", srv.response.cloud.points.size());
	else
	  printf("Service call failed\n");
	return 0;
}