#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <cybird_detector/DetectorConfig.h>
#include <cybird_detector/Detection.h>
#include <opencv2/core.hpp>
#include <vector>
#include "opencv2/aruco/dictionary.hpp"

class CBDetector {
	public:
	CBDetector();
	~CBDetector();

	private:
	ros::NodeHandle _node;
	ros::Subscriber _cam_sub;
	ros::Publisher _cam_pub;
	ros::Publisher _det_pub;
	cv::Mat _dist_matrix;
	cv::Mat _cam_matrix;
	dynamic_reconfigure::Server<cybird_detector::DetectorConfig> _server;
	void parse_cam_params(sensor_msgs::CameraInfo ros_cam);
	void config_callback(cybird_detector::DetectorConfig &new_config, int level);
	void image_callback(const sensor_msgs::Image& msg);
};
