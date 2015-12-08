/*!
 * \file
 * \brief 
 * \author Patryk Wasowski
 */

#ifndef REDDISKDETECTION_HPP_
#define REDDISKDETECTION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/Int16.h"


namespace Processors {
namespace RedDiskDetection {

/*!
 * \class RedDiskDetection
 * \brief RedDiskDetection processor class.
 *
 * Component detects red disk what is essential for finding out which rod is starting one.
 */
class RedDiskDetection: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	RedDiskDetection(const std::string & name = "RedDiskDetection");

	/*!
	 * Destructor
	 */
	virtual ~RedDiskDetection();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
    Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

    ros::Publisher pub;
    ros::NodeHandle *nh;

	// Properties
	Base::Property<std::string> ros_topic_name;
	Base::Property<std::string> ros_namespace;

	
	// Handlers
	void onNewImg();

};

} //: namespace RedDiskDetection
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RedDiskDetection", Processors::RedDiskDetection::RedDiskDetection)

#endif /* REDDISKDETECTION_HPP_ */
