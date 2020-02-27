#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class cloudHandler
{
	public:
	
	cloudHandler()
	{
		pcl_sub=nh.subscribe("/camera/depth_registered/points",1,&cloudHandler::cloudCB, this);
		pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("pcl_color",1);		
	}
	
	void cloudCB(const sensor_msgs::PointCloud2 &input)
	{
		//creat PCL clouds
		pcl::PointCloud<pcl::PointXYZ> cloud; //original cloud	
		pcl::PointCloud<pcl::PointXYZRGB> color_cloud; //imagen a color
		
		//ROS to PCL
		pcl::fromROSMsg(input,cloud);
		
		color_cloud.points.resize(cloud.size());
		for(size_t i=0;i=color_cloud.points.size();i++)
		{
			color_cloud.points[i].x=cloud.points[i].x;
			color_cloud.points[i].x=cloud.points[i].y;
			color_cloud.points[i].x=cloud.points[i].z;
			if(i==200 || i==201 || i==2002 || i==2003 || i==2004 || i==2005 || i==2006 || i==2006 || i==2008 || i==2009 || i==210)
			{
				color_cloud.points[i].r=0;
				color_cloud.points[i].g=255;
				color_cloud.points[i].b=0;
			}
			else
			{
				color_cloud.points[i].r=255;
				color_cloud.points[i].g=0;
				color_cloud.points[i].b=0;				
			}
		}
		
		//transforms PCL to ROS and publish
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(color_cloud,output);
		output.header.frame_id="camera_link";
		pcl_pub.publish(output);			
	}
	
	protected:
		ros::NodeHandle nh;
		ros::Subscriber pcl_sub;
		ros::Publisher pcl_pub;
};

main(int argc, char** argv)
{
	ros::init(argc,argv,"figProyect");
	cloudHandler handler;
	ros::spin();
	return 0;
}
