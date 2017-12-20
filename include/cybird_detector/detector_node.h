#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <cybird_detector/DetectorConfig.h>
#include <cybird_detector/Detection.h>
#include <cybird_detector/CalibrateCamera.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include "opencv2/aruco.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
	std::map<int, float> _size_mapping;
	double _fovx;
	double _fovy;
	int _active_id;
	int _frame_skip;
	int _curr_frame;
	dynamic_reconfigure::Server<cybird_detector::DetectorConfig> _server;

	void draw_vectors(cv::Mat &in, cv::Scalar color, int line_width, int voffset,
		int id, double x, double y, double distance, double cx, double cy);
	void parse_size_mapping(std::string mapping_str);
	void parse_cam_params(sensor_msgs::CameraInfo ros_cam);
	bool calibrate_cam(cybird_detector::CalibrateCamera::Request &req,
		cybird_detector::CalibrateCamera::Response &res);
	void config_callback(cybird_detector::DetectorConfig &new_config, int level);
	void image_callback(const sensor_msgs::Image& msg);
};
