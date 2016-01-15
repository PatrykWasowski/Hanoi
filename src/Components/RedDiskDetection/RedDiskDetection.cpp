/*!
 * \file
 * \brief
 * \author Patryk Wasowski
 */

#include <memory>
#include <string>
#include <iostream>

#include "RedDiskDetection.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace RedDiskDetection {

RedDiskDetection::RedDiskDetection(const std::string & name) :
		Base::Component(name) , 
        ros_topic_name("ros.topic_name", std::string("red_disk")),
        ros_namespace("ros.namespace", std::string("discode")) {
	registerProperty(ros_topic_name);
	registerProperty(ros_namespace);

    min_h = 7;
    max_h = 10;

    min_y = 230;
    max_y = 830;

    min_area = 200;
}

RedDiskDetection::~RedDiskDetection() {
}

void RedDiskDetection::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
    registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImg", boost::bind(&RedDiskDetection::onNewImg, this));
	addDependency("onNewImg", &in_img);

}

bool RedDiskDetection::onInit() {
    static char * tmp = NULL;
    static int tmpi;
    ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle;
    pub = nh->advertise<std_msgs::Int16>(ros_topic_name, 1000); // drugi argument to wielkość kolejki

	return true;
}

bool RedDiskDetection::onFinish() {
	return true;
}

bool RedDiskDetection::onStop() {
	return true;
}

bool RedDiskDetection::onStart() {
	return true;
}

void RedDiskDetection::onNewImg() {
    std_msgs::Int16 msg;

    if(!in_img.empty())
    {
        cv::Mat src, gray, hsv, thresh, img;
        std::vector<cv::Mat> hsv_split;
        cv::Point p;

        src = in_img.read().clone();
        int rows_poprawka = 350;
        int cols_poprawka = -44;

        int rows = src.rows / 2 + rows_poprawka;
        int cols = src.cols / 2 + cols_poprawka;

        cv::Point src_center(cols, rows); // punkt znajdujący się idealnie pod chwytakiem, gdy ramię jest 12cm nad podłożem

        cv::cvtColor(src, gray, CV_BGR2GRAY);
        cv::cvtColor(src, hsv, CV_BGR2HSV);

        split(hsv, hsv_split);

        cv::inRange(hsv_split[0], min_h, max_h, thresh);
        cv::blur(thresh, thresh, cv::Size(5,5));
        //cv::erode(thresh, thresh, cv::Mat());

/*      Wykrywanie kółek średnio wychodzi, więc spróbuję wykrywać kontury i na tej podstawie wykryć, gdzie jest czerwony krążek
*/

        img = src.clone();
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Point> contours_poly;
        cv::Rect boundRect;
        cv::Mat cont;
        thresh.copyTo(cont);
        cv::findContours( cont, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        int max = 0, i_cont = -1;

        for( int i = 0; i< contours.size(); i++ )
        {
            if ( abs(cv::contourArea(cv::Mat(contours[i]))) > max )
            {
                cv::approxPolyDP( cv::Mat(contours[i]), contours_poly, 3, true );
                boundRect = cv::boundingRect( cv::Mat(contours_poly) );
                int row = (boundRect.tl().y + boundRect.br().y) / 2;
                if(row > min_y && row < max_y)
                {
                    max = abs(cv::contourArea(cv::Mat(contours[i])));
                    i_cont = i;
                }
             }
        }
        if ( i_cont >= 0 )
        {
            cv::approxPolyDP( cv::Mat(contours[i_cont]), contours_poly, 3, true );
            boundRect = cv::boundingRect( cv::Mat(contours_poly) );
            cv::fillConvexPoly(img, contours_poly, contours_poly.size() );
            cv::rectangle( src, boundRect.tl(), boundRect.br(), cv::Scalar(125, 250, 125), 2, 8, 0 );
            cv::line( src, boundRect.tl(), boundRect.br(), cv::Scalar(250, 125, 125), 2, 8, 0);
            cv::line( src, cv::Point(boundRect.x + boundRect.width, boundRect.y), cv::Point(boundRect.x, boundRect.y + boundRect.height), cv::Scalar(250, 125, 125), 2, 8, 0);
            drawContours(thresh,  contours, i_cont, cv::Scalar(125, 125, 250), 2 );

            int row = (boundRect.tl().y + boundRect.br().y) / 2;
            int col = (boundRect.tl().x + boundRect.br().x) / 2;
            p = cv::Point(col, row);

            cv::circle(src, p, 3, cv::Scalar(0, 255, 255), 3, 8, 0);

            //std::cout << "x."
        }




        // void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
                       //thickness – Thickness of the circle outline, if positive. Negative thickness means that a filled circle is to be drawn.
                       //lineType – Type of the circle boundary. See the line() description.
                       //shift – Number of fractional bits in the coordinates of the center and in the radius value.
        circle(src, src_center, 3, cv::Scalar(255, 255, 0), 3, 8, 0);

        std::cout << src_center.x << " - src_center.x   " << p.x << " - p.x\n";

        int roznica = src_center.x - p.x;

        std::cout << "Roznica " << src_center.x - p.x << "\n";

        msg.data = roznica;

        pub.publish(msg);

        out_img.write(src);
    }
}



} //: namespace RedDiskDetection
} //: namespace Processors
