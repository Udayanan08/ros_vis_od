#include "../include/visual_odometry/vis_od.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


class vis_od : public rclcpp::Node{
	public:
	vis_od():Node("visual_odometry"){
		// Subscription to the left image topic
		left_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                        "stereo_left", 10, std::bind(&vis_od::topic_callback, this, std::placeholders::_1));

		// Subscription to the right image topic
		right_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                        "stereo_right", 10, std::bind(&vis_od::topic_callback2, this, std::placeholders::_1));

		// Image publisher
	  	img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("vis_od/Img", 10);
		timer2_ = this->create_wall_timer(
      	3ms, std::bind(&vis_od::timer_callback2, this));

	  	// cap.open(0);
		// cap.set(CAP_PROP_FPS, 60);
		// cap.set(CAP_PROP_FRAME_WIDTH, 320);
		// cap.set(CAP_PROP_FRAME_HEIGHT, 240);
		
	}

	private:
	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg){
		left_frame = cv_bridge::toCvShare(msg, "bgr8")->image;

		left_img_flag = true;

	}
	void topic_callback2(const sensor_msgs::msg::Image::SharedPtr msg){
		right_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		right_img_flag = true;
	}



		// cap.read(frame);
		// cout<<"frame size : ";
		// cout<<frame.size()<<endl;
		// left_frame = frame(cv::Range(0, frame.size().height), cv::Range(0, frame.size().width/2));
		// right_frame = frame(cv::Range(0, frame.size().height), cv::Range(frame.size().width/2, frame.size().width));
		
		// match_image(left_frame, right_frame, projMat1, projMat2, match_img2);
		// auto msg = sensor_msgs::msg::Image();
		// msg.encoding = "mono8"; // assuming BGR color spaces
		// msg.height = match_img2.rows;
		// msg.width = match_img2.cols;
		// msg.step = match_img2.step;
		// size_t size = match_img2.rows * match_img2.step;
		// msg.data.resize(size);
		// memcpy(msg.data.data(), match_img2.data, size);

		// // Publish the image message
		// match_img_publisher->publish(msg);
	
	void timer_callback2(){
		// cap.read(frame);
		// left_frame = frame(cv::Range(0, frame.size().height), cv::Range(0, frame.size().width/2));
		// right_frame = frame(cv::Range(0, frame.size().height), cv::Range(frame.size().width/2, frame.size().width));
		// cv::cvtColor(left_frame, dist_gray_frame, cv::COLOR_BGR2GRAY);
		// cv::cvtColor(right_frame, dist_gray_frame2, cv::COLOR_BGR2GRAY);
		
		// cv::undistort(dist_gray_frame, match_img2, projMat1.cam, projMat1.dist, noArray());
		// cv::undistort(dist_gray_frame2, gray_frame2, projMat2.cam, projMat2.dist, noArray());

		
		if (left_img_flag ==true && right_img_flag ==true){
			match_image(left_frame, right_frame, projMat1, projMat2, match_img2);

			match_img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", match_img2)
               .toImageMsg();

			// auto msg = sensor_msgs::msg::Image();
			// msg.encoding = "mono8"; // assuming BGR color space
			// msg.height = match_img2.rows;
			// msg.width = match_img2.cols;
			// msg.step = match_img2.step;
			// // msg.header.stamp = this->now();
			// size_t size = match_img2.rows * match_img2.step;
			// msg.data.resize(size);
			// memcpy(msg.data.data(), match_img2.data, size);

			// Publish the image message
			img_publisher_->publish(*match_img_msg_.get());
			left_img_flag = false;
			right_img_flag = false;
		}
	}
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_img_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_img_subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
	sensor_msgs::msg::Image::SharedPtr match_img_msg_;
	rclcpp::TimerBase::SharedPtr timer2_;

	bool right_img_flag, left_img_flag;
};



void match_image(Mat& left_frame, Mat& right_frame, calib_data& projMat1, calib_data& projMat2, Mat& match_img2){
	cv::cvtColor(left_frame, dist_gray_frame, cv::COLOR_BGR2GRAY);
	cv::cvtColor(right_frame, dist_gray_frame2, cv::COLOR_BGR2GRAY);

	cv::undistort(dist_gray_frame, gray_frame, projMat1.cam, projMat1.dist, noArray());
	cv::undistort(dist_gray_frame2, gray_frame2, projMat2.cam, projMat2.dist, noArray());

	vector<DMatch> good_matches;
	fdetectMatch(gray_frame, gray_frame2, kp, kp2, good_matches);
	cout<<"goodd matches size - "<<good_matches.size()<<endl;
	Mat R,t;
	//posecomp(kp, kp2, good_matches, projMat1, projMat2, R, t);
			
	drawMatches(gray_frame, kp, gray_frame2, kp2, good_matches, match_img2, Scalar::all(-1),
 	Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<vis_od>());
	rclcpp::shutdown();
	return 0;	
	
}


