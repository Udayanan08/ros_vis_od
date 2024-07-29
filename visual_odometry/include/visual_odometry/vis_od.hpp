#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "feature.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cv_bridge/cv_bridge.h>



Mat left_frame_new = Mat();
Mat right_frame_new = Mat();
Mat left_frame = Mat();
Mat right_frame = Mat();
Mat left_frame1 = Mat();
Mat right_frame1 = Mat();
Mat dist_gray_frame, dist_gray_frame2, dist_gray_frame3;
Mat gray_frame, gray_frame2, gray_frame3;
VideoCapture cap;
Mat match_img, match_img2;
Mat R,t;


vector<KeyPoint> kp;

vector<KeyPoint> kp2;

Mat points4d;
vector<Point3d> points;
vector<DMatch> good_matches;

void match_image(Mat& , Mat&, Mat&, calib_data& , calib_data&, Mat&);

//getting projection matrices from calibration file
YAML::Node config1 = YAML::LoadFile("install/visual_odometry/lib/config/usb3_left.yaml");
//Mat projMat1 = read_yaml(config1["projection_matrix"]);
calib_data projMat1 = read_yaml2(config1["camera_matrix"], config1["distortion_coefficients"], config1["projection_matrix"]);
YAML::Node config2 = YAML::LoadFile("install/visual_odometry/lib/config/usb3_right.yaml");
//Mat projMat2 = read_yaml(config2["projection_matrix"]);
calib_data projMat2 = read_yaml2(config2["camera_matrix"], config2["distortion_coefficients"], config2["projection_matrix"]);
