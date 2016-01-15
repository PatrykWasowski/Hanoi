/*!
 * \file
 * \brief
 * \author Patryk Wasowski
 */

#include <memory>
#include <string>
#include <iostream>


#include "FindPiece.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace FindPiece {

FindPiece::FindPiece(const std::string & name) :
		Base::Component(name) , 
        ros_topic_name("ros.topic_name", std::string("int")),
        ros_namespace("ros.namespace", std::string("discode")) {
	registerProperty(ros_topic_name);
	registerProperty(ros_namespace);

}

FindPiece::~FindPiece() {
}

void FindPiece::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImg", boost::bind(&FindPiece::onNewImg, this));
	addDependency("onNewImg", &in_img);

}

bool FindPiece::onInit() {
    static char * tmp = NULL;
    static int tmpi;
    ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle;
    pub = nh->advertise<std_msgs::Float32MultiArray>(ros_topic_name, 1000);

	return true;
}

bool FindPiece::onFinish() {
	return true;
}

bool FindPiece::onStop() {
	return true;
}

bool FindPiece::onStart() {
	return true;
}

void FindPiece::onNewImg() {

    std_msgs::Float32MultiArray msg;


    if(!in_img.empty())
    {
		cv::Mat gray, src;
        src = in_img.read().clone();
        
        int rows = src.rows / 2 + 190; // poprawka na pozycje kamery
        //std::cout << rows << std::endl;
        int cols = src.cols / 2 - 30;
		//std::cout << cols << std::endl;
		
        cv::Point src_center(cols, rows);
        

        cv::cvtColor(src, gray, CV_RGB2GRAY);
        //cv::cvtColor(src, src, CV_BGR2RGB);
        //cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

        std::vector<cv::Vec3f> circles;
	
        cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 200, 100, 0, 0);

        for( size_t i = 0; i < circles.size(); i++ )
          {
              cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
              int radius = cvRound(circles[i][2]);
              // circle center
              circle( src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
              // circle outline
              circle( src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
              
              
              msg.data.push_back((float) (src_center.x - center.x)/5000.0);
              msg.data.push_back((float) (src_center.y - center.y)/3500.0);
              
              //std::cout << "dx : " << (src_center.x - center.x)/5000.0 << std::endl;
              //std::cout << "dy : " << (src_center.y - center.y)/3500.0 << std::endl;
              
           }
           
        circle(src, src_center, 3, cv::Scalar(255, 0 ,0), 3, 8, 0);

        out_img.write(gray);
        if(circles.size() == 0)
            msg.data.push_back(0.0);

        std::cout << circles.size() << "\n";

    }
    else
        msg.data.push_back(-1.0);

    pub.publish(msg);
}



} //: namespace FindPiece
} //: namespace Processors
