#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <opencv2/opencv.hpp> 
#include <algorithm>
#include <vector>
#include <string>

class Detect {
public:

	struct Paths {
		cv::Size desc_size;
		std::vector<std::string> paths;

		Paths() = default;

		Paths(cv::Size desc_size_, std::vector<std::string> paths_):
			desc_size(desc_size_),
			paths(paths_)
		{
		}
	};

	Detect() = default;

	Detect(Paths paths) {
		load(paths);
	}

	void load(Paths paths) {
		hog.winSize = paths.desc_size;

		models.clear();
		for(unsigned long i = 0; i < paths.paths.size(); i++)
			models.push_back(cv::ml::SVM::load(paths.paths[i]));
	}

	std::vector<float> compute(cv::Mat& input) {
		std::vector<float> results;

		cv::Mat proc = proc_img(input);
		for(unsigned long i = 0; i < models.size(); i++) 
			results.push_back(models[i]->predict(proc));
		return results;
	}

private:
	cv::Size desc_size;

	cv::HOGDescriptor hog;
	std::vector<cv::Ptr<cv::ml::SVM>> models;

	cv::Mat proc_img(cv::Mat& input) {
		cv::Mat proc;
		cv::resize(input, proc, hog.winSize);

		std::vector<float> descriptors;
		hog.compute(proc, descriptors, cv::Size(8, 8));

		proc.create(1, descriptors.size(), CV_32FC1);
		cv::transpose(cv::Mat(descriptors), proc);

		return proc;
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_detect");
	ros::NodeHandle nh("~");

	ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("signs", 25);
	std_msgs::UInt8MultiArray msg;

	ros::Time begin = ros::Time::now();
	ros::Duration duration(1.0/25.0);

	std::vector<std::string> signs_path;
	nh.getParam("signs_model", signs_path);

	const std::vector<cv::Scalar> colors = { cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255) };
	Detect::Paths paths(cv::Size(64, 64), signs_path);
	Detect signs_model(paths);

	const int cap_id = 0;
	cv::VideoCapture cap(cap_id);
	if(!cap.isOpened()) {
		ROS_FATAL("can not open web camera number %d\n", cap_id);
		return 1;
	}

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(cv::CAP_PROP_FPS, 30);

	cv::Mat frame, gray, prop;
	std::vector<cv::Point> approx;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<float> pred_values;

	while(cap.read(frame)) {	
		cv::blur(frame, gray, cv::Size(3, 3));

		cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
		cv::Canny(gray, prop, 30, 180, 3);
        
		cv::findContours(prop, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		for(size_t i = 0; i < contours.size(); i++) {
			cv::approxPolyDP(contours[i], approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.01, true);

			cv::Rect rectangle = cv::boundingRect(approx);
			float dl = (float)rectangle.width / (float)rectangle.height;
			if(dl < 0.7 || dl > 1.3 || rectangle.width < 20 || rectangle.height < 20)
				continue;
			
			cv::Mat roi = gray(rectangle);
			pred_values = signs_model.compute(roi);
			for(unsigned long i = 0; i < pred_values.size(); i++) {
				if(pred_values[i] > 0.3) {
					cv::rectangle(frame, rectangle, colors[i], 2);
					if(std::find(std::begin(msg.data), std::end(msg.data), i) == std::end(msg.data)) 
						msg.data.push_back(i);
				}
			}
		}


		cv::imshow("frame", frame);
		cv::imshow("canny", prop);
		if(cv::waitKey(10) == 27) {
			break;
		}

		if((ros::Time::now() - begin) >= duration) {
			pub.publish(msg);
			msg.data.clear();
		}
	}

	return 0;
}
