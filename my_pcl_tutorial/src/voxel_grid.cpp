#include <ros/ros.h>
//PCL inlcudes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
	pcl::PCLPointCloud2 cloud_filtered;
	
	//preform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.1,0.1,0.1);
	sor.filter (cloud_filtered);
	
	//publish the data
	pub.publish (cloud_filtered);
}

int main (int argc, char** argv)
{
	//Initial ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	
	//create a ros susbscriber
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
	
	//create a ros publisher
	pub=nh.advertise<pcl::PCLPointCloud2> ("KVoutput", 1);
	
	//spin
	ros::spin();
}
