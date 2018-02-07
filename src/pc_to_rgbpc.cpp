#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class pcToRGBPc
{
public:
  pcToRGBPc()
    : it_(nh_)
  {
  	gotCamInfo = false;
  	recPc = recIm = false;

  	fromFrame = "velodyne";
  	toFrame = "camera_optical";

    imSub = it_.subscribe("color_image", 1, &pcToRGBPc::imCB, this);
    infoSub = nh_.subscribe("camera_info", 1, &pcToRGBPc::infoCB, this);
    pcSub = nh_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, &pcToRGBPc::pcCB, this);
    
    pcPub = nh_.advertise<sensor_msgs::PointCloud2>("colored_point_cloud", 1);
    updateTimer = nh_.createTimer(ros::Duration(1/20), &pcToRGBPc::updateCB, this);

  }	

  void updateCB(const ros::TimerEvent& event)
  {
  	if(recPc && recIm && gotCamInfo)
  	{
  		ROS_WARN("In here 2");
  		recPc = false;
  		recPc = recIm = false;
	  	cv::Mat image;
	    cv_bridge::CvImagePtr input_bridge;
	    
	    try {
	      input_bridge = cv_bridge::toCvCopy(latestIm, sensor_msgs::image_encodings::BGR8);
	      image = input_bridge->image;
	    }
	    catch (cv_bridge::Exception& ex){
	      ROS_ERROR("[draw_frames] Failed to convert image");
	      return;
	    }

    if(updateTf())
    {
    	ROS_WARN("In here 4");
  		for(int i = 0; i < latestPC.points.size(); i++)
  		{
  			pcl::PointXYZRGB pt = latestPC.points.at(i);
      	 	tf::Vector3 ptInCam(pt.x, pt.y, pt.z);

      	 	tf::Vector3 ptInWorld = transform.inverse() * ptInCam;

      	 	cv::Point3d pt_cv(ptInWorld.x(), ptInWorld.y(), ptInWorld.z());
      		cv::Point2d uv;
      		uv = cam_model_.project3dToPixel(pt_cv);

	  		uint8_t r = 0, g = 0, b = 0;    // Example: Red color
      		if(uv.x >= 0 && uv.y >= 0 && uv.x < input_bridge->image.cols && uv.y < input_bridge->image.rows)
      		{
      			r = image.at<cv::Vec3b>(uv.y,uv.x)[0];
      			g = image.at<cv::Vec3b>(uv.y,uv.x)[1];
      			b = image.at<cv::Vec3b>(uv.y,uv.x)[2];
      		}
	       	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	 	    latestPC.points.at(i).rgb = rgb;
  		}
  		
  		sensor_msgs::PointCloud2 cloud;
  		pcl::toROSMsg(latestPC, cloud);
  		ROS_INFO("Publishing");
  		pcPub.publish(cloud);
  	}	
  }
}

  bool updateTf()
  {
		tf::StampedTransform _transform;
		try {
			tf_listener_.lookupTransform(fromFrame, toFrame, ros::Time(0), _transform);
			transform = _transform;
			return true;
		}
		catch (tf::TransformException& ex) {
			ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
			return false;
		}

  }

  void pcCB(const sensor_msgs::PointCloud2::ConstPtr& _msg)
  {
  	ROS_WARN("I heard PC");
  	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(*_msg,pcl_pc);
  	pcl::fromPCLPointCloud2(pcl_pc, latestPC);
  	recPc = true;
  }


 void imCB(const sensor_msgs::ImageConstPtr& _msg)
 {
 	ROS_WARN("I heard Image");
 	latestIm = *_msg;
 	recIm = true;
 }

  void infoCB(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
  	ROS_WARN("I heard camera");
  	if(!gotCamInfo){
  		gotCamInfo = true;
    	cam_model_.fromCameraInfo(info_msg);
  	}
  }
  
  private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber imSub;
  ros::Subscriber infoSub;
  ros::Subscriber pcSub;
  ros::Publisher pcPub;
  ros::Timer updateTimer;
 
  tf::Transform transform;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;

  pcl::PointCloud<pcl::PointXYZRGB> latestPC;
  sensor_msgs::Image latestIm;

  std::string fromFrame, toFrame;
  bool gotCamInfo;
  bool recPc;
  bool recIm;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  pcToRGBPc converter;
  ros::spin();
  return 0;
}