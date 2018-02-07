#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>

class savImgPcd{
public:
	savImgPcd() : it_(p_nh)
	{
		imSub = it_.subscribe("image_rect", 1, &savImgPcd::imCB, this);
		trigger = p_nh.subscribe<std_msgs::Empty>("save_img_pcd", 1, &savImgPcd::triggerCB, this);
		pcSub = p_nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, &savImgPcd::pcCB, this);
		ROS_INFO("All set up. Waiting for data");
	}

	~savImgPcd()
	{	
		cv::destroyWindow(WINDOW_NAME);
	}

private:
	ros::NodeHandle p_nh;
	image_transport::ImageTransport it_;
	sensor_msgs::Image latestIm;
	sensor_msgs::PointCloud2 latestPC;

	ros::Subscriber trigger;
	image_transport::Subscriber imSub;
	ros::Subscriber pcSub;

	const std::string WINDOW_NAME = "image";

	void imCB(const sensor_msgs::ImageConstPtr& _msg)
	{
		latestIm = *_msg;
	}

	void pcCB(const sensor_msgs::PointCloud2::ConstPtr& _msg)
	{
		latestPC = *_msg;
	}

	void triggerCB(const std_msgs::Empty trigger)
	{
		pcl::PCDWriter writer;
		cv_bridge::CvImagePtr cv_ptr;

		try{
			cv_ptr = cv_bridge::toCvCopy(latestIm, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception:[%s]", e.what());
			return;
		}

		pcl::PCLPointCloud2 pcl_pc;
		pcl_conversions::toPCL(latestPC,pcl_pc);

	  	pcl::PointCloud<pcl::PointXYZ> cloud;
	  	pcl::fromPCLPointCloud2(pcl_pc, cloud);
		ROS_INFO("Point cloud time stamp:[%f]", latestPC.header.stamp.toSec());

		writer.write("/home/abhi/ws/catkin_ws/src/auro/data/file.pcd", pcl_pc);

		ROS_INFO("Saved image time stamp:[%f]", cv_ptr->header.stamp.toSec());
		cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
		cv::imshow(WINDOW_NAME, cv_ptr->image);

		imwrite( "/home/abhi/ws/catkin_ws/src/auro/data/file.png", cv_ptr->image );

		cv::waitKey(0);
		//TODO: save image and point cloud as image and pcd file respectively
	}

};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_img_pcd");
	savImgPcd _saver;
	ros::spin();

	return 0;
}