/*!
 * \file
 * \brief
 * \author Patryk Wasowski
 */

#include <memory>
#include <string>

#include "RodsDetection.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace RodsDetection {

int wanted_rod = -1;

RodsDetection::RodsDetection(const std::string & name) :
		Base::Component(name) , 
        ros_topic_name("ros.topic_name", std::string("rods")),
        ros_namespace("ros.namespace", std::string("discode")) {
	registerProperty(ros_topic_name);
	registerProperty(ros_namespace);

}

RodsDetection::~RodsDetection() {
}

void RodsDetection::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImg", boost::bind(&RodsDetection::onNewImg, this));
	addDependency("onNewImg", &in_img);

}

void callback(const std_msgs::String& str){
    if(str.data == "LEFT")
        wanted_rod = 0;
    else if(str.data == "RIGHT")
        wanted_rod = 1;
    else
        wanted_rod = 2;
}

bool RodsDetection::onInit() {
    static char * tmp = NULL;
    static int tmpi;
    ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle;
    nhs = new ros::NodeHandle;
    sub = nh->subscribe("wanted_rod", 1, callback);
    //pub = nh->advertise<std_msgs::Int16>(ros_topic_name, 1000); // drugi argument to wielkość kolejki

	return true;
}

bool RodsDetection::onFinish() {
	return true;
}

bool RodsDetection::onStop() {
	return true;
}

bool RodsDetection::onStart() {
	return true;
}

void RodsDetection::onNewImg() {
    if(!in_img.empty())
    {
        cv::Mat src, hsv, thresh;
        std::vector<cv::Mat> hsv_split;
        src = in_img.read().clone();

        int rows_poprawka = 350;
        int cols_poprawka = -44;

        int rows = src.rows / 2 + rows_poprawka;
        int cols = src.cols / 2 + cols_poprawka;

        cv::Point src_center(cols, rows);

        cv::cvtColor(src, hsv, CV_BGR2HSV);
        int min_h = 330, max_h = 345;   // minimalna i maksymalna wartosc skladowej H dla koloru rozowego

        split(hsv, hsv_split);  // podzial obrazka na macierze z 3 skladowymi H, S, V

        cv::inRange(hsv_split[0], min_h, max_h, thresh);
        cv::blur(thresh, thresh, cv::Size(5,5));
        //cv::erode(thresh, thresh, cv::Mat());

        std::vector<cv::Vec3f> circles;
        int min_distance = 200; // minimalna odleglosc pomiedzy wykrytymi kolkami
        int min_radius = 10, max_radius = 30;


        cv::HoughCircles(thresh, circles, CV_HOUGH_GRADIENT, 1, min_distance, 200, 100, min_radius, max_radius);
        for(int i = 0; i < circles.size(); ++i)
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int rad = cvRound(circles[i][2]);

            cv::circle(src, center, 3, cv::Scalar(255, 0, 0), -1, 8, 0);
            cv::circle(src, center, rad, cv::Scalar(0, 255, 0), 3, 8, 0);
        }

        circle(src, src_center, 3, cv::Scalar(255, 255, 0), 3, 8, 0);

        out_img.write(src);
    }
}



} //: namespace RodsDetection
} //: namespace Processors
