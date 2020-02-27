/* this code made a downsample, planar segmentation and in that way i got 
 * 4 clouds, just the plane without objects, just the objects without plane
 * and both with the cloud normal, as I recived by ros, then a cloud just
 * with the plane, a cloud just with objects, but both from the cloud downsampled,
 * also I publish the normal cloud, and the downsampled cloud; and the last cloud 
 * is the result of a ortogonal projection of the figures points*/
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>


class cloudHandler
{
	public:
	
	cloudHandler()
	{
		pcl_sub=nh.subscribe("/camera/depth/points",1,&cloudHandler::cloudCB, this);
		pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("pcl_segmented",1);
		pcl_pub2=nh.advertise<sensor_msgs::PointCloud2>("pcl_segmented2",1);
		pcl_pub3=nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled",1);
		pcl_pub4=nh.advertise<sensor_msgs::PointCloud2>("pcl_segmDown",1);
		pcl_pub5=nh.advertise<sensor_msgs::PointCloud2>("pcl_segmDown2",1);
		pcl_pub6=nh.advertise<sensor_msgs::PointCloud2>("pcl_cloud",1);
		pcl_pub7=nh.advertise<sensor_msgs::PointCloud2>("pcl_projected",1);
		ind_pub=nh.advertise<pcl_msgs::PointIndices>("point_indices",1);
		coef_pub=nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef",1);
		ind_pub2=nh.advertise<pcl_msgs::PointIndices>("point_indices2",1);
		coef_pub2=nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef2",1);
	}
	
	void cloudCB(const sensor_msgs::PointCloud2 &input)
	{
		//creat PCL clouds
		pcl::PointCloud<pcl::PointXYZ> cloud; //original cloud
		pcl::PointCloud<pcl::PointXYZ> cloud_segmented; //just plane 
		pcl::PointCloud<pcl::PointXYZ> cloud_segmented2; // without plane
		pcl::PointCloud<pcl::PointXYZ> cloud_downsampled; // with voxel
		pcl::PointCloud<pcl::PointXYZ> cloud_segmDown; // just plane with voxel
		pcl::PointCloud<pcl::PointXYZ> cloud_segmDown2; // without plane with voxel	
		pcl::PointCloud<pcl::PointXYZ> cloud_projected; //cloud with figure projected	
		
		//ROS to PCL
		pcl::fromROSMsg(input,cloud);
		
		//downsampling PCL cloud 
		pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
		voxelSampler.setInputCloud(cloud.makeShared());
		voxelSampler.setLeafSize(0.01f,0.01f,0.01f);
		voxelSampler.filter(cloud_downsampled);
		
		//create PCL coefficients and inliers 
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
		
		//set PCL process to cloud without downsampling
		pcl::SACSegmentation<pcl::PointXYZ> segmentation;
		segmentation.setModelType(pcl::SACMODEL_PLANE);
		segmentation.setMethodType(pcl::SAC_RANSAC);
		segmentation.setMaxIterations(1000);
		segmentation.setDistanceThreshold(0.05);
		segmentation.setInputCloud(cloud.makeShared());
		segmentation.segment(*inliers,*coefficients);
		
		//set PCL process to cloud with downsampling
		pcl::SACSegmentation<pcl::PointXYZ> segmentation2;
		segmentation2.setModelType(pcl::SACMODEL_PLANE);
		segmentation2.setMethodType(pcl::SAC_RANSAC);
		segmentation2.setMaxIterations(1000);
		segmentation2.setDistanceThreshold(0.05);
		segmentation2.setInputCloud(cloud_downsampled.makeShared());
		segmentation2.segment(*inliers2,*coefficients2);
		
		//conversion PCL to ROS coefficients and inliers
		pcl_msgs::ModelCoefficients ros_coefficients;
		pcl_conversions::fromPCL(*coefficients, ros_coefficients);
		ros_coefficients.header.stamp=input.header.stamp;
		coef_pub.publish(ros_coefficients);
		
		pcl_msgs::PointIndices ros_inliers;
		pcl_conversions::fromPCL(*inliers,ros_inliers);
		ros_inliers.header.stamp=input.header.stamp;
		ind_pub.publish(ros_inliers);
		
		pcl_msgs::ModelCoefficients ros_coefficients2;
		pcl_conversions::fromPCL(*coefficients2, ros_coefficients2);
		ros_coefficients2.header.stamp=input.header.stamp;
		coef_pub2.publish(ros_coefficients2);
		
		pcl_msgs::PointIndices ros_inliers2;
		pcl_conversions::fromPCL(*inliers2,ros_inliers2);
		ros_inliers2.header.stamp=input.header.stamp;
		ind_pub2.publish(ros_inliers2);
		
		//filters extract without downsampling
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud.makeShared());
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(cloud_segmented);
		extract.setNegative(true);
		extract.filter(cloud_segmented2);
		
		//filters extract with downsampling 
		pcl::ExtractIndices<pcl::PointXYZ> extract2;
		extract2.setInputCloud(cloud_downsampled.makeShared());
		extract2.setIndices(inliers2);
		extract2.setNegative(false);
		extract2.filter(cloud_segmDown);
		extract2.setNegative(true);
		extract2.filter(cloud_segmDown2);
		
		//project objects in the plane
		pcl::ProjectInliers<pcl::PointXYZ> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);
		proj.setInputCloud (cloud_segmDown2.makeShared());
		proj.setModelCoefficients(coefficients2);
		proj.filter(cloud_projected);
		
		//transforms PCL to ROS and publish
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_segmented,output);
		output.header.frame_id="camera_link";
		pcl_pub.publish(output);
		
		sensor_msgs::PointCloud2 output2;
		pcl::toROSMsg(cloud_segmented2,output2);
		output2.header.frame_id="camera_link";
		pcl_pub2.publish(output2);
		
		sensor_msgs::PointCloud2 output3;
		pcl::toROSMsg(cloud_downsampled,output3);
		output3.header.frame_id="camera_link";
		pcl_pub3.publish(output3);
		
		sensor_msgs::PointCloud2 output4;
		pcl::toROSMsg(cloud_segmDown,output4);
		output4.header.frame_id="camera_link";
		pcl_pub4.publish(output4);
		
		sensor_msgs::PointCloud2 output5;
		pcl::toROSMsg(cloud_segmDown2,output5);
		output5.header.frame_id="camera_link";
		pcl_pub5.publish(output5);
		
		sensor_msgs::PointCloud2 output6;
		pcl::toROSMsg(cloud,output6);
		output6.header.frame_id="camera_link";
		pcl_pub6.publish(output6);
		
		sensor_msgs::PointCloud2 output7;
		pcl::toROSMsg(cloud_projected,output7);
		output7.header.frame_id="camera_link";
		pcl_pub7.publish(output7);
		
	}
	
	protected:
	
		ros::NodeHandle nh;
		ros::Subscriber pcl_sub;
		ros::Publisher pcl_pub, ind_pub, coef_pub, ind_pub2, coef_pub2, pcl_pub2, pcl_pub3, pcl_pub4, pcl_pub5, pcl_pub6, pcl_pub7;
};

main(int argc, char** argv)
{
	ros::init(argc,argv,"figProyect");
	cloudHandler handler;
	ros::spin();
	return 0;
}
