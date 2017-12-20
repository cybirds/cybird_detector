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

	ROS_DEBUG("Parsing size mapping...");
	string mapping_str;
	_node.getParam("size_mapping", mapping_str);
	parse_size_mapping(mapping_str);
}

CBDetector::~CBDetector()
{
}

/**
 * Parses size mapping string into usable map
 */
void CBDetector::parse_size_mapping(std::string mapping_str)
{
	stringstream ss(mapping_str);
	vector<string> size_vec;
	string single_size;
	while(std::getline(ss, single_size, ',')) {
		size_vec.push_back(single_size);
	}
	for(std::string const& size : size_vec) {
		auto ix = size.find(':');
		_size_mapping[atoi(size.substr(0,ix).data())] = atof(size.substr(ix+1).data());
	}
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

/**
 * Changes configuration dynamically, currently no options supported 
 */
void CBDetector::config_callback(cybird_detector::DetectorConfig &new_config, int level)
{
	ROS_INFO("Reconfigure received!");
}

/**
 * Detects markers, estimates pose, calculates distance and offsets
 */
void CBDetector::image_callback(const sensor_msgs::Image& msg)
{
	// Setup output vectors and predefined marker dictionary
	// TODO: cv::Ptr<cv::aruco::DetectorParameters> parameters;
	vector<int> marker_ids;
	vector< vector<cv::Point2f> > marker_corners;
	cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);

	// Convert ROS Image msg to OpenCV Mat
	cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	// Detect markers
	cv::aruco::detectMarkers(img_ptr->image, dictionary, marker_corners, marker_ids);

	// Estimate pose and draw markers
	for(int i=0; i < marker_ids.size(); i++) {
		// Only continue if size mapping exists
		int curr_id = marker_ids[i];
		if(_size_mapping.find(curr_id) != _size_mapping.end()) {
			// OpenCV aruco assumes all markers are same size, so must estimate pose one marker at a time
			vector<cv::Vec3d> rvecs, tvecs;
			vector<int> single_id = {curr_id};
			vector< vector<cv::Point2f> > single_corner = {marker_corners[i]};
			cv::aruco::estimatePoseSingleMarkers(single_corner, _size_mapping[curr_id], _cam_matrix, _dist_matrix, rvecs, tvecs);
			cv::aruco::drawDetectedMarkers(img_ptr->image, single_corner, single_id);
			cv::aruco::drawAxis(img_ptr->image, _cam_matrix, _dist_matrix, rvecs[0], tvecs[0], 0.01);
		}
	}
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
