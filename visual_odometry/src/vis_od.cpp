#include "../include/visual_odometry/vis_od.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


class vis_od : public rclcpp::Node{
	public:
	vis_od(cv::String file_name):Node("visual_odometry"),file_name(file_name){
		// Subscription to the left image topic
		left_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                        "stereo_left", 10, std::bind(&vis_od::topic_callback, this, std::placeholders::_1));

		// Subscription to the right image topic
		right_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                        "stereo_right", 10, std::bind(&vis_od::topic_callback2, this, std::placeholders::_1));

		// Image publisher
	  	img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("vis_od/Img", 10);
		timer2_ = this->create_wall_timer(
      	100ms, std::bind(&vis_od::timer_callback2, this));
		
	}

	private:
	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg){
		left_frame_new = cv_bridge::toCvShare(msg, "mono8")->image;
		left_frame1 = left_frame.clone();
		left_frame = left_frame_new.clone();
		left_img_flag = true;
	}
	void topic_callback2(const sensor_msgs::msg::Image::SharedPtr msg){
		right_frame_new = cv_bridge::toCvShare(msg, "mono8")->image;
		right_frame1 = right_frame.clone();
		right_frame = right_frame_new.clone();
		right_img_flag = true;
	}
	
	void timer_callback2(){
		if (left_img_flag ==true && right_img_flag ==true){
			if(!left_frame1.empty()){
				auto start = std::chrono::high_resolution_clock::now();
				match_image(left_frame, right_frame, left_frame1, projMat1, projMat2, match_img2,file_name);
				match_img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", match_img2).toImageMsg();
				auto end = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
				cout << "The function took " << duration.count() << " milliseconds\n";
		// 		Publish the image message
				img_publisher_->publish(*match_img_msg_.get());
				left_img_flag = false;
				right_img_flag = false;
				cout<<"count : "<<count<<endl;
				count++;
			}
		}
	}
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_img_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_img_subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
	sensor_msgs::msg::Image::SharedPtr match_img_msg_;
	rclcpp::TimerBase::SharedPtr timer2_;
	cv::String file_name;
	bool right_img_flag, left_img_flag;
	int count;
};

void match_image(Mat& left_frame, Mat& right_frame, Mat& left_frame1, calib_data& projMat1, calib_data& projMat2, Mat& match_img2, String& file_name){
	// cv::cvtColor(left_frame, dist_gray_frame, cv::COLOR_BGR2GRAY);
	// cv::cvtColor(right_frame, dist_gray_frame2, cv::COLOR_BGR2GRAY);
	// cv::cvtColor(left_frame1, dist_gray_frame3, cv::COLOR_BGR2GRAY);

	// cv::undistort(left_frame, gray_frame, projMat1.cam, projMat1.dist, noArray());
	// cv::undistort(right_frame, gray_frame2, projMat2.cam, projMat2.dist, noArray());
	// cv::undistort(left_frame1, gray_frame3, projMat1.cam, projMat1.dist, noArray());

	// imshow("hello", left_frame);
    // waitKey(0);
	vector<DMatch> good_matches;
	fdetectMatch(left_frame, right_frame, left_frame1, projMat1.cam, R, t, match_img2);
	comp_path(R,t,R_trans,t_trans,trans_mat);
	ofstream file(file_name, std::ios::app);
	if(file.is_open()){
		trans_flat = trans_mat.reshape(1,1);
		for(int i=0; i<12;i++){
			file<<trans_flat.at<double>(0,i)<<" ";
		}
		file<<endl;
		file.close();
	}
	
	// cout<<"Rotation matarix : "<<endl<<R<<endl;
	// cout<<"Translational matrix : "<<endl<<trans_mat<<endl;
	R.release();
	t.release();
}

void comp_path(Mat& R, Mat& t, Mat& R_trans, Mat& t_trans, Mat& trans_mat){
	R_trans = R * R_trans;
	t_trans = t + t_trans;
	augment(R_trans, t_trans, trans_mat);
	
}

void augment(Mat& R, Mat& t, Mat& trans_mat){
	trans_mat.at<double>(0,0)=R.at<double>(0,0);
	trans_mat.at<double>(0,1)=R.at<double>(0,1);
	trans_mat.at<double>(0,2)=R.at<double>(0,2);
	trans_mat.at<double>(0,3)=t.at<double>(0,0);
	trans_mat.at<double>(1,0)=R.at<double>(1,0);
	trans_mat.at<double>(1,1)=R.at<double>(1,1);
	trans_mat.at<double>(1,2)=R.at<double>(1,2);
	trans_mat.at<double>(1,3)=t.at<double>(0,1);
	trans_mat.at<double>(2,0)=R.at<double>(2,0);
	trans_mat.at<double>(2,1)=R.at<double>(2,1);
	trans_mat.at<double>(2,2)=R.at<double>(2,2);
	trans_mat.at<double>(2,3)=t.at<double>(0,2);

}


int main(int argc, char * argv[]){

	cv::String f_n = argv[1];
	cv::String file_name = f_n + ".txt";
	YAML::Node config1 = YAML::LoadFile("install/visual_odometry/lib/config/kitti_gray.yaml");
	YAML::Node config2 = YAML::LoadFile("install/visual_odometry/lib/config/kitti_gray.yaml");
	projMat1.cam = read_yaml_kitti(config1["P0"]);
	projMat2.cam = read_yaml_kitti(config2["P1"]);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<vis_od>(file_name));
	rclcpp::shutdown();
	return 0;	
}


