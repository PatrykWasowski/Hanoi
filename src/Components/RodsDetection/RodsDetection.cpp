/*!
 * \file
 * \brief
 * \author Patryk Wasowski
 */

#include <memory>
#include <string>
#include <iostream>
#include "RodsDetection.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace RodsDetection {

int wanted_rod = -1;

RodsDetection::RodsDetection(const std::string & name) :
		Base::Component(name) , 
        ros_topic_name("ros.topic_name", std::string("rods")),
        ros_namespace("ros.namespace", std::string("IRpOS")) {
	registerProperty(ros_topic_name);
	registerProperty(ros_namespace);

    min_h = 6;
    max_h = 9;

    min_y = 350;
    max_y = 920;

    min_area = 600;
    max_area = 3200;

    px_cm_x = 50;
    px_cm_y = 60;

    wanted_rod = -1;
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


bool RodsDetection::onInit() {
    static char * tmp = NULL;
    static int tmpi;
    ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle;
    nhs = new ros::NodeHandle;
    pub = nhs->advertise<std_msgs::Int32MultiArray>(ros_topic_name, 1000); // drugi argument to wielkość kolejki

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
        //std::cout << '[RODS_DETECTION]' << "\n";
        
		cv::Mat src, gray, hsv, thresh, img;
        std::vector<cv::Mat> hsv_split;
        cv::Point p;

        std_msgs::Int32MultiArray ret_msg;    // wiadomosc do skryptu Python zawierajaca odleglosc punktu pod chwytakiem do zadanego slupka

        src = in_img.read().clone();
        int rows_poprawka = 370;
        int cols_poprawka = -54;

        int rows = src.rows / 2 + rows_poprawka;
        int cols = src.cols / 2 + cols_poprawka;

        cv::Point src_center(cols, rows); // punkt znajdujący się idealnie pod chwytakiem, gdy ramię jest 12cm nad podłożem

        cv::cvtColor(src, gray, CV_BGR2GRAY);
        cv::cvtColor(src, hsv, CV_BGR2HSV);

        split(hsv, hsv_split);

        cv::inRange(hsv_split[0], min_h, max_h, thresh);
        //cv::blur(thresh, thresh, cv::Size(5,5));
        //cv::erode(thresh, thresh, cv::Mat());
/*
    Wykrywanie kółek średnio wychodzi, więc spróbuję wykrywać kontury i na tej podstawie wykryć, gdzie jest czerwony krążek
*/
        img = src.clone();
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Point> contours_poly;
        cv::Rect boundRect;
        cv::Mat cont;
        thresh.copyTo(cont);
        cv::findContours( cont, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        contour* areas;
        areas = new contour[contours.size()];

        for( int i = 0; i< contours.size(); ++i )
        {
             areas[i].cont = contours[i];
             areas[i].area = cv::contourArea(cv::Mat(contours[i]));

             // Jeli jeden z wykrytych konturow jest podejrzanie duzy i prostokat obramowujacy go
             // jest gdzies na dolnej lub gornej krawedzi widzianego obrazu
             // to jest to pomaranczowa blacha obramowujaca tasme, wiec mozna ten wykryty kontur olac dajac mu powierzchnie 0

             if(areas[i].area > min_area && areas[i].area < max_area)
             {
                 cv::approxPolyDP( cv::Mat(areas[i].cont), contours_poly, 3, true );
                 boundRect = cv::boundingRect( cv::Mat(contours_poly) );
                 int row = (boundRect.tl().y + boundRect.br().y) / 2;
                 if(row > max_y || row < min_y)
                     areas[i].area = 0;
                 else
                     areas[i].rect = boundRect;
             }
             else
                    areas[i].area = 0;
        }
        
        //std::cout << contours.size() << "\n";
        
        int znalezione = 0;
        sort(areas, areas + contours.size());
        for(int i = 0; i < 4; ++i)
        {
            //std::cout << areas[i].area << "\n";
            if(areas[i].area > 0)
                ++znalezione;
        }

        //std::cout << "Znaleziono  " << znalezione << "\n";
        contour rods[3];
        for(int i = 0; i < 3; ++i)
        {
            if(znalezione == 4)
                rods[i] = areas[i+1];
            else rods[i] = areas[i];
            //std::cout << "Rod[" << i <<"]=" << rods[i].area;
        }

		// wysylanie info do topicu /rods
        
		

        for(int i = 0; i < 3; ++i)
        {
            if(rods[i].area > 0)
            {
                // narysowanie prostokąta obramowującego znaleziony słupek
                
                cv::fillConvexPoly(img, contours_poly, contours_poly.size() );
                cv::rectangle( src, rods[i].rect.tl(), rods[i].rect.br(), cv::Scalar(125, 250, 125), 2, 8, 0 );
                cv::line( src, rods[i].rect.tl(), rods[i].rect.br(), cv::Scalar(250, 125, 125), 2, 8, 0);
                cv::line( src, cv::Point(rods[i].rect.x + rods[i].rect.width, rods[i].rect.y), cv::Point(rods[i].rect.x, rods[i].rect.y + rods[i].rect.height), cv::Scalar(250, 125, 125), 2, 8, 0);
                drawContours(thresh,  contours, i, cv::Scalar(125, 125, 250), 2 );

                int row = (rods[i].rect.tl().y + rods[i].rect.br().y) / 2;
                int col = (rods[i].rect.tl().x + rods[i].rect.br().x) / 2;
                p = cv::Point(col, row);

                cv::circle(src, p, 3, cv::Scalar(0, 255, 255), 3, 8, 0);

				// dołożenie do ret_msg współrzędnych znalezionego słupka
				
				ret_msg.data.push_back(row);	// odcięta
				ret_msg.data.push_back(col);	// rzędna
            }
        }

		delete[] areas; 	

        circle(src, src_center, 3, cv::Scalar(255, 255, 0), 3, 8, 0);

		ret_msg.data.push_back(src_center.y);
		ret_msg.data.push_back(src_center.x);

		pub.publish(ret_msg);
        out_img.write(src);
    }
}



} //: namespace RodsDetection
} //: namespace Processors
