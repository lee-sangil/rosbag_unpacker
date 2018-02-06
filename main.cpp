#include "common.h"
#include "parser.h"
#include "Thirdparty/DLib/FileFunctions.h"

#include "ros/ros.h"
#include "ros/package.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

class Communicator{
	public:
		Communicator():it(nh_){
			if( !Parser::hasOption("-i") || !Parser::hasOption("-o") ){
				std::cout << "Mandatory -i: /Path/to/input.bag.\n"
					"Mandatory -o: /Path/to/directory/.\n"
					"Example: rosrun rosbag_unpacker rosbag_unpacker -i /path/to/input.bag -o /path/to/directory/" << std::endl;
				return;
			}

			if( std::system((std::string("mkdir -p ") + Parser::getOption("-o") + std::string("rgb")).c_str()) == -1 ){
				ROS_ERROR("Cannot make directory");
				return;
			}

			if( std::system((std::string("mkdir -p ") + Parser::getOption("-o") + std::string("depth")).c_str()) == -1 ){
				ROS_ERROR("Cannot make directory");
				return;
			}

			file_rgb.open(Parser::getOption("-o") + std::string("rgb.txt"));
			file_depth.open(Parser::getOption("-o") + std::string("depth.txt"));
			file_caminfo.open(Parser::getOption("-o") + std::string("camera.yaml"));

			file_rgb << "# color images" << std::endl << "# file" << std::endl << "# timestamp filename" << std::endl;
			file_depth << "# depth images" << std::endl << "# file" << std::endl << "# timestamp filename" << std::endl;
			file_caminfo << "# Camera name: " << std::endl;
			is_logged_caminfo = false;

			sub_rgb = it.subscribe( Parser::getStringOption("-ti", "/camera/rgb/image_color"), 1, &Communicator::callback_rgb, this);
			sub_depth = it.subscribe( Parser::getStringOption("-td", "/camera/depth/image"), 1, &Communicator::callback_depth, this);
		}
		void callback_rgb(const sensor_msgs::ImageConstPtr& msg_rgb);
		void callback_depth(const sensor_msgs::ImageConstPtr& msg_depth);

	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it;
		image_transport::Subscriber sub_rgb;
		image_transport::Subscriber sub_depth;
		std::ofstream file_rgb;
		std::ofstream file_depth;
		std::ofstream file_caminfo;
		bool is_logged_caminfo;
};

void Communicator::callback_rgb(const sensor_msgs::ImageConstPtr& msg_rgb){
	cv_bridge::CvImagePtr img_ptr_rgb;

	try{
		img_ptr_rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream ss;
	ss << Parser::getStringOption("-o") << "rgb/" << msg_rgb->header.stamp << ".png";

	cv::Mat& color = img_ptr_rgb->image;
	cv::imwrite(ss.str(), color);
	file_rgb << msg_rgb->header.stamp << "rgb/" << msg_rgb->header.stamp << ".png" << std::endl;
}

void Communicator::callback_depth(const sensor_msgs::ImageConstPtr& msg_depth){
	cv_bridge::CvImagePtr img_ptr_depth;

	try{
		img_ptr_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	std::stringstream ss;
	ss << Parser::getStringOption("-o") << "depth/" << msg_depth->header.stamp << ".png";

	cv::Mat& depth = img_ptr_depth->image;
	cv::imwrite(ss.str(), depth);
	file_depth << msg_depth->header.stamp << "depth/" << msg_depth->header.stamp << ".png" << std::endl;
}

int main(int argc, char * argv[]){

	ros::init(argc, argv, "rosbag_unpacker");
	Parser::init(argc, argv);
	Communicator comm_;

	ros::spin();

	return 0;
}
