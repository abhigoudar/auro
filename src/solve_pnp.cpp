#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
 
#include <iostream>
#include <string>
#include <cmath>  
 
std::vector<cv::Point2f> Generate2DPoints();
std::vector<cv::Point3f> Generate3DPoints();

void toEuler(cv::Mat rotationMatrix)
{

  double yaw, roll, pitch;
  int solution_number = 1;

  double m00 = rotationMatrix.at<double>(0,0);
  double m01 = rotationMatrix.at<double>(0,1);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m21 = rotationMatrix.at<double>(2,1);
  double m22 = rotationMatrix.at<double>(2,2);

      struct Euler
    {
        double yaw;
        double pitch;
        double roll;
    };

    Euler euler_out;
    Euler euler_out2; //second solution
                      //get the pointer to the raw data

                      // Check that pitch is not at a singularity
                      // Check that pitch is not at a singularity
    if (abs(m20) >= 1)
    {
        euler_out.yaw = 0;
        euler_out2.yaw = 0;

        // From difference of angles formula
        if (m20 < 0)  //gimbal locked down
        {
            double delta = atan2(m01, m02);
            euler_out.pitch = CV_PI / double(2.0);
            euler_out2.pitch = CV_PI / double(2.0);
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
        else // gimbal locked up
        {
            double delta = atan2(-m01, -m02);
            euler_out.pitch = -CV_PI / double(2.0);
            euler_out2.pitch = -CV_PI / double(2.0);
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
    }
    else
    {
        euler_out.pitch = -asin(m20);
        euler_out2.pitch = CV_PI - euler_out.pitch;

        euler_out.roll = atan2(m21 / cos(euler_out.pitch), m22 / cos(euler_out.pitch));
        euler_out2.roll = atan2(m21 / cos(euler_out2.pitch), m22 / cos(euler_out2.pitch));

        euler_out.yaw = atan2(m10 / cos(euler_out.pitch), m00 / cos(euler_out.pitch));
        euler_out2.yaw = atan2(m10 / cos(euler_out2.pitch), m00 / cos(euler_out2.pitch));
    }

    if (solution_number == 1)
    {
        yaw = euler_out.yaw;
        pitch = euler_out.pitch;
        roll = euler_out.roll;
    }
    else
    {
        yaw = euler_out2.yaw;
        pitch = euler_out2.pitch;
        roll = euler_out2.roll;
    }

  std::cout << "Value of angles:YPR " << yaw << " " << pitch << " " << roll << std::endl;
//  return euler;
}

 
int main( int argc, char* argv[])
{
  ros::init(argc, argv, "solve_pnp");
  ros::NodeHandle nh;

  // Read points
  std::vector<cv::Point2f> imagePoints = Generate2DPoints();
  std::vector<cv::Point3f> objectPoints = Generate3DPoints();
 
  std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
  cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
  //cv::setIdentity(cameraMatrix);

  cameraMatrix.at<double>(0,0) = 484.181682;
  cameraMatrix.at<double>(0,1) = 0.000000;
  cameraMatrix.at<double>(0,2) = 456.751937;
  cameraMatrix.at<double>(1,0) = 0.000000;
  cameraMatrix.at<double>(1,1) = 484.525727;
  cameraMatrix.at<double>(1,2) = 364.568142;
  cameraMatrix.at<double>(2,0) = 0.000000;
  cameraMatrix.at<double>(2,1) = 0.000000;
  cameraMatrix.at<double>(2,2) = 1.000000;

  std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;
 
  cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = -0.199205;
  distCoeffs.at<double>(1) = 0.067711;
  distCoeffs.at<double>(2) = 0.003373;
  distCoeffs.at<double>(3) = -0.000042;
  distCoeffs.at<double>(4) = 0.0000;

  cv::Mat rvec(3,1,cv::DataType<double>::type);
  cv::Mat tvec(3,1,cv::DataType<double>::type);
 
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false);
  
  std::cout << "rvec: " << rvec << std::endl;
  std::cout << "tvec: " << tvec << std::endl;

  cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, true, 10000, 8.0);

  std::cout << "rvec: " << rvec << std::endl;
  std::cout << "tvec: " << tvec << std::endl;
  
  cv::Mat rotCamerMatrix;
  cv::Rodrigues(rvec,rotCamerMatrix);

  toEuler(rotCamerMatrix);
  
  
  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

 
  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }
 
  return 0;
}
 
 
std::vector<cv::Point2f> Generate2DPoints()
{
  std::vector<cv::Point2f> points;
 
  float x,y;
 
  x=303;y=323;
  points.push_back(cv::Point2f(x,y));
 
  x=298;y=433;
  points.push_back(cv::Point2f(x,y));
 
  x=493;y=332;
  points.push_back(cv::Point2f(x,y));
 
  x=491;y=438;
  points.push_back(cv::Point2f(x,y));
 
  // x=424;y=298;
  // points.push_back(cv::Point2f(x,y));

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
 
  return points;
}
 
 
std::vector<cv::Point3f> Generate3DPoints()
{
  std::vector<cv::Point3f> points;
 
 
  float x,y,z;
 
  x=1.56262934208;y=0.165110349655;z=-0.0771114081144;
  points.push_back(cv::Point3f(x,y,z));
 
  x=1.59982609749;y=0.172602042556;z=-0.374975770712;
  points.push_back(cv::Point3f(x,y,z));
 
  x=1.54179024696;y=-0.38135266304;z=-0.0841651856899;
  points.push_back(cv::Point3f(x,y,z));
 
  x=1.58272171021;y=-0.376264870167;z=-0.380912393332;
  points.push_back(cv::Point3f(x,y,z));

  // x=1.71779704094;y=-0.181591778994;z=0.150899201632;
  // points.push_back(cv::Point3f(x,y,z));
   
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
 
  return points;
}