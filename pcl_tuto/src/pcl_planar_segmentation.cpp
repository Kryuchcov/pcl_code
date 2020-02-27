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
		pcl_pub7=nh.advertise<sensor_msgs::PointCloud2>("radius_cloud",1);
		pcl_pub8=nh.advertise<sensor_msgs::PointCloud2>("cluster_cloud",1);
		pcl_pub9=nh.advertise<sensor_msgs::PointCloud2>("circle_cloud",1);
		ind_pub=nh.advertise<pcl_msgs::PointIndices>("point_indices",1);
		coef_pub=nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef",1);
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
		pcl::PointCloud<pcl::PointXYZ> radius_cloud; // radius filter
		pcl::PointCloud<pcl::PointXYZ> radius_cloud2; // radius filter
		pcl::PointCloud<pcl::PointXYZ> circle_cloud; // cloud with the circle
		//point cloud pointer to cluster process
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer (new pcl::PointCloud<pcl::PointXYZ>);
		
		
		//ROS to PCL
		pcl::fromROSMsg(input,cloud);
		
		//downsampling PCL cloud 
		pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
		voxelSampler.setInputCloud(cloud.makeShared());
		voxelSampler.setLeafSize(0.01f,0.01f,0.01f);
		voxelSampler.filter(cloud_downsampled);
		
		//create PCL coefficients and inliers 
		pcl::ModelCoefficients coefficients;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::ModelCoefficients coefficients2;
		pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
		pcl::ModelCoefficients coefficients3;
		pcl::PointIndices::Ptr inliers3(new pcl::PointIndices());
		
		//set PCL process to cloud without downsampling
		pcl::SACSegmentation<pcl::PointXYZ> segmentation;
		segmentation.setModelType(pcl::SACMODEL_PLANE);
		segmentation.setMethodType(pcl::SAC_RANSAC);
		segmentation.setMaxIterations(1000);
		segmentation.setDistanceThreshold(0.05);
		segmentation.setInputCloud(cloud.makeShared());
		segmentation.segment(*inliers,coefficients);
		
		//set PCL process to cloud with downsampling
		pcl::SACSegmentation<pcl::PointXYZ> segmentation2;
		segmentation2.setModelType(pcl::SACMODEL_PLANE);
		segmentation2.setMethodType(pcl::SAC_RANSAC);
		segmentation2.setMaxIterations(1000);
		segmentation2.setDistanceThreshold(0.05);
		segmentation2.setInputCloud(cloud_downsampled.makeShared());
		segmentation2.segment(*inliers2,coefficients2);
		
		//conversion PCL to ROS coefficients and inliers
		pcl_msgs::ModelCoefficients ros_coefficients;
		pcl_conversions::fromPCL(coefficients, ros_coefficients);
		ros_coefficients.header.stamp=input.header.stamp;
		coef_pub.publish(ros_coefficients);
		
		pcl_msgs::PointIndices ros_inliers;
		pcl_conversions::fromPCL(*inliers,ros_inliers);
		ros_inliers.header.stamp=input.header.stamp;
		ind_pub.publish(ros_inliers);
		
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
		extract2.filter(*cloud_pointer);
		
		//Centroid object
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(cloud_segmDown2,centroid);
		
		//radius filter 
		float resolution=128.0f;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
		octree.setInputCloud(cloud_segmDown2.makeShared());
		octree.addPointsFromInputCloud();
		pcl::PointXYZ center_point;
		center_point.x=centroid[0];
		center_point.y=centroid[1];
		center_point.z=centroid[2];
		float radius=1;
		std::vector<int> radiusIdx;
		std::vector<float>radiusSQDist;
		if(octree.radiusSearch(center_point,radius,radiusIdx,radiusSQDist)>0)
		{
			for(size_t i=0;i<radiusIdx.size();i++)
			{
				radius_cloud.points.push_back(cloud_segmDown2.points[radiusIdx[i]]);
			}
		}
		
		//cluster the point cloud
		//creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_pointer);
		
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.01); //2cm-->0.02
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_pointer);
		ec.extract(cluster_indices);
		
		//writer pcl to save clouds as pcd
		pcl::PCDWriter writer;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		
		int i=0;
		for(std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();it++)
		{
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();++pit)
				cloud_cluster->points.push_back (cloud_pointer->points[*pit]);
			cloud_cluster->width=cloud_cluster->points.size();
			cloud_cluster->height=1;
			cloud_cluster->is_dense=true;
			
			
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			std::stringstream ss;
			ss << "cloud_cluster_" << i << ".pcd";
			writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
			i++;
		}
		
		//cicle segmentation
		pcl::SACSegmentation<pcl::PointXYZ> circleSeg;
		circleSeg.setModelType(pcl::SACMODEL_CIRCLE2D);
		circleSeg.setMethodType(pcl::SAC_RANSAC);
		circleSeg.setMaxIterations(1000);
		circleSeg.setDistanceThreshold(0.05);
		circleSeg.setInputCloud(cloud_segmDown2.makeShared());
		circleSeg.segment(*inliers3,coefficients3);
		
		//extract circle
		pcl::ExtractIndices<pcl::PointXYZ> extractCircle;
		extractCircle.setInputCloud(cloud_segmDown2.makeShared());
		extractCircle.setIndices(inliers3);
		extractCircle.setNegative(false);
		extractCircle.filter(circle_cloud);
		
		
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
		pcl::toROSMsg(radius_cloud,output7);
		output7.header.frame_id="camera_link";
		pcl_pub7.publish(output7);
		
		sensor_msgs::PointCloud2 output8;
		pcl::toROSMsg(*cloud_cluster,output8);
		output8.header.frame_id="camera_link";
		pcl_pub8.publish(output8);
		
		sensor_msgs::PointCloud2 output9;
		pcl::toROSMsg(circle_cloud,output9);
		output9.header.frame_id="camera_link";
		pcl_pub9.publish(output9);
		
	}
	
	protected:
	
		ros::NodeHandle nh;
		ros::Subscriber pcl_sub;
		ros::Publisher pcl_pub, ind_pub, coef_pub, pcl_pub2, pcl_pub3, pcl_pub4, pcl_pub5, pcl_pub6, pcl_pub7, pcl_pub8, pcl_pub9;
};

main(int argc, char** argv)
{
	ros::init(argc,argv,"pcl_planar_segmentation");
	cloudHandler handler;
	ros::spin();
	return 0;
}
