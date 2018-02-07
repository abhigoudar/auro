#! /usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

def yamlToCameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def imageCallback(data):
    camera_info_msg.header = data.header    
    publisher.publish(camera_info_msg)

if __name__ == "__main__":
    rospy.init_node("camera_info_publisher", anonymous=True)
    
    # Get fname from command line (cmd line input required)
    filename = ""
    if rospy.has_param("~calib_file") :
        filename = rospy.get_param("~calib_file")
    else:
        rospy.logerr("[%s]: Could not find camera calibration parameter.", rospy.get_name())

    # Parse yaml file
    camera_info_msg = yamlToCameraInfo(filename)

    # Initialize publisher node
    publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    rospy.Subscriber("image_color", Image, imageCallback)
    
    rospy.spin()
