#include "cybird_detector/detector_node.h"
#include <math.h>
#include <iostream>

using namespace std;

CBDetector::CBDetector() :
	_node("cybird_detector"),
	_dist_matrix(1, 5, CV_32FC1),
	_cam_matrix(3, 3, CV_32FC1)
{
	ROS_DEBUG("Setting up dynamic reconfiguration server");
	_server.setCallback(boost::bind(&CBDetector::config_callback, this, _1, _2));

	ROS_DEBUG("Setting up publishers and subscribers");
	_cam_sub = _node.subscribe("/webcam/image_raw", 30, &CBDetector::image_callback, this);
	_cam_pub = _node.advertise<sensor_msgs::Image>("detection_image", 30);
	_det_pub = _node.advertise<cybird_detector::Detection>("detection", 30);

	ROS_DEBUG("Receiving camera calibration info...");
	sensor_msgs::CameraInfo ros_cam_param = 
		*(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/webcam/camera_info", _node));
	parse_cam_params(ros_cam_param);
}

CBDetector::~CBDetector()
{
}

/**
 * Converts ROS CameraInfo into Cv::Mat objects
 */
void CBDetector::parse_cam_params(sensor_msgs::CameraInfo ros_cam)
{
	// Grab K vector and convert to 3x3 intrinsics matrix
	for(int row=0; row < _cam_matrix.rows; row++) {
		for(int col=0; col < _cam_matrix.cols; col++) {
			_cam_matrix.at<float>(row, col) = ros_cam.K[(row*3)+col];
		}
	}

	// Copy distortion vector
	for(int col=0; col < _dist_matrix.cols; col++) {
		_dist_matrix.at<float>(0, col) = ros_cam.D[col];
	}

	ROS_INFO_STREAM("Camera intrinsic matrix - " << endl << _cam_matrix << endl << 
		"Camera distortion matrix - " << endl << _dist_matrix << endl);
}

void CBDetector::config_callback(cybird_detector::DetectorConfig &new_config, int level)
{
	ROS_INFO("Reconfigure received!");
}

void CBDetector::image_callback(const sensor_msgs::Image& msg)
{
	cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	//cv::Ptr<cv::aruco::DetectorParameters> parameters;
	cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
	vector<int> markerIds;
	vector< vector<cv::Point2f> > markerCorners;
	cv::aruco::detectMarkers(img_ptr->image, dictionary, markerCorners, markerIds);
	cv::aruco::drawDetectedMarkers(img_ptr->image, markerCorners, markerIds);
	_cam_pub.publish(img_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cybird_detector");
	CBDetector detector;

	ROS_INFO("Starting cybird_detector");
	ros::spin();
	return 0;
}
