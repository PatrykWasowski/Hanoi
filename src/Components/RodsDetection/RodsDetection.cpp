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

    min_h = 7;
    max_h = 12;

    min_y = 550;
    max_y = 1000;

    min_area = 150;
    max_area = 3000;

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

void callback(const std_msgs::String& str){
    if(str.data == "LEFT")
        wanted_rod = 0;
    else if(str.data == "CENTER")
        wanted_rod = 1;
    else
        wanted_rod = 2;
        
    std::cout << "\n\n\n\nOdebralem zadanie\n\n\n\n\n\n";
}

bool RodsDetection::onInit() {
    static char * tmp = NULL;
    static int tmpi;
    ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle;
    nhs = new ros::NodeHandle;
    sub = nh->subscribe("wanted_rod", 1, callback);
    pub = nhs->advertise<std_msgs::Float32MultiArray>(ros_topic_name, 1000); // drugi argument to wielkość kolejki

	ros::spinOnce();

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
    ros::spinOnce();
    
    if(!in_img.empty())
    {
        //std::cout << '[RODS_DETECTION]' << "\n";
        
		cv::Mat src, gray, hsv, thresh, img;
        std::vector<cv::Mat> hsv_split;
        cv::Point p;

        std_msgs::Float32MultiArray ret_msg;    // wiadomosc do skryptu Python zawierajaca odleglosc punktu pod chwytakiem do zadanego slupka

        src = in_img.read().clone();
        int rows_poprawka = 370;
        int cols_poprawka = -75;

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

        //std::cout<<"Flag-1\n";
        int max_col = -1;
        int min_col = 1000;
        int found = 0;  // ile znaleziono slupkow

        //std::cout << "Znaleziono  " << znalezione << "\n";
        contour rods[3];
        for(int i = 0; i < 3; ++i)
        {
            if(znalezione == 4)
                rods[i] = areas[i+1];
            else rods[i] = areas[i];
            //std::cout << "Rod[" << i <<"]=" << rods[i].area;
        }


        for(int i = 0; i < 3; ++i)
        {
            if(rods[i].area > 0)
            {
                cv::fillConvexPoly(img, contours_poly, contours_poly.size() );
                cv::rectangle( src, rods[i].rect.tl(), rods[i].rect.br(), cv::Scalar(125, 250, 125), 2, 8, 0 );
                cv::line( src, rods[i].rect.tl(), rods[i].rect.br(), cv::Scalar(250, 125, 125), 2, 8, 0);
                cv::line( src, cv::Point(rods[i].rect.x + rods[i].rect.width, rods[i].rect.y), cv::Point(rods[i].rect.x, rods[i].rect.y + rods[i].rect.height), cv::Scalar(250, 125, 125), 2, 8, 0);
                drawContours(thresh,  contours, i, cv::Scalar(125, 125, 250), 2 );

                int row = (rods[i].rect.tl().y + rods[i].rect.br().y) / 2;
                int col = (rods[i].rect.tl().x + rods[i].rect.br().x) / 2;
                p = cv::Point(col, row);

                cv::circle(src, p, 3, cv::Scalar(0, 255, 255), 3, 8, 0);

                ++found;
            }

        }

        float wanted_x = -3000, wanted_y = -3000;     // wartosci x i y zadanego slupka, ktore beda wyslane do topicu

        // wysylanie info, w jakiej odleglosci od punktu pod chwytakiem jest zadany slupek

        //std::cout << "Flag0\n";

        switch(wanted_rod)
        {
            case 0:

                    for(int i = 0; i < 3; ++i)
                    {
                        int col = (rods[i].rect.tl().x + rods[i].rect.br().x) / 2;
                        int row = (rods[i].rect.tl().y + rods[i].rect.br().y) / 2;

                        if(rods[i].area && col < min_col)
                        {
                            min_col = col;
                            wanted_x = col;
                            wanted_y = row;
							if(rods[i].area > 1000)
									wanted_y += 20;	// jesli wykryty jest czerwony krazek, to punkt, w ktorym trzeba zlapac jest troche nizej
                        }
                    }
            break;

            case 1:
                   if(found == 3)
                   {
                       // znalezienie minimum i maksimum
                       for(int i = 0; i < 3; ++i)
                       {
                           int col = (rods[i].rect.tl().x + rods[i].rect.br().x) / 2;

                           if(col < min_col)
                               min_col = col;

                           else if(col > max_col)
                               max_col = col;
                       }
                       // wybor slupka, ktory nie jest w zadnym ekstremum skladowej x
                       
                       std::cout << "min-max col: " << min_col << " - " << max_col << "\n";
                       for(int i = 0; i < 3; ++i)
                       {
                           int col = (rods[i].rect.tl().x + rods[i].rect.br().x) / 2;
                           int row = (rods[i].rect.tl().y + rods[i].rect.br().y) / 2;
						   std::cout << "col[" << i << "]: " << col << "\n"; 
                           if(col < max_col && col > min_col)
                           {
                               wanted_x = col;
                               wanted_y = row;
							   if(rods[i].area > 1000)
									wanted_y += 20;	// jesli wykryty jest czerwony krazek, to punkt, w ktorym trzeba zlapac jest troche nizej
                           }
                           
                       }
                   }
              break;

              case 2:
                    for(int i = 0; i < 3; ++i)
                    {
                        int col = (rods[i].rect.tl().x + rods[i].rect.br().x) / 2;
                        int row = (rods[i].rect.tl().y + rods[i].rect.br().y) / 2;

                        if(rods[i].area && col > max_col)
                        {
                            max_col = col;
                            wanted_x = col;
                            wanted_y = row;
                            if(rods[i].area > 1000)
									wanted_y += 20;	// jesli wykryty jest czerwony krazek, to punkt, w ktorym trzeba zlapac jest troche nizej
                        }
                    }
              break;

        }

		delete[] areas; 	

        circle(src, src_center, 3, cv::Scalar(255, 255, 0), 3, 8, 0);

        //std::cout << "Flag1\n";
        if(wanted_rod > -1 && wanted_x != -3000)
        {	
			std::cout << "wanted_rod: " << wanted_x << " - " << wanted_y << "\n";
			std::cout << "src_center: " << src_center.x << " - " << src_center.y << "\n";
            wanted_x -= src_center.x;  // otrzymujemy pozycje wzgledem punktu pod chwytakiem
            wanted_y -= src_center.y;

			std::cout << "wanted_rod - src: " << wanted_x << " - " << wanted_y << "\n";

            wanted_x /= px_cm_x;    // zwroci odleglosc w cm, nie pikselach
            wanted_y /= px_cm_y;

            wanted_x /= 100;    // trzeba wyrazic odleglosc w m dla irpos.move
            wanted_y /= 100;

            ret_msg.data.push_back(wanted_x);
            ret_msg.data.push_back(wanted_y);
            pub.publish(ret_msg);
            std::cout << "msg: " << ret_msg.data[0] << " " << ret_msg.data[1] << "\n";
       }
        out_img.write(src);
        
    }

}



} //: namespace RodsDetection
} //: namespace Processors
