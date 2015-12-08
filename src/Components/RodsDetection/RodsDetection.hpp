/*!
 * \file
 * \brief 
 * \author Patryk Wasowski
 */

#ifndef RODSDETECTION_HPP_
#define RODSDETECTION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace Processors {
namespace RodsDetection {

/*!
 * \class RodsDetection
 * \brief RodsDetection processor class.
 *
 * Component for detecting rods visible for camera
 */
class RodsDetection: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	RodsDetection(const std::string & name = "RodsDetection");

	/*!
	 * Destructor
	 */
	virtual ~RodsDetection();

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
    ros::Subscriber sub;
    ros::NodeHandle *nh, *nhs;

	// Properties
	Base::Property<std::string> ros_topic_name;
	Base::Property<std::string> ros_namespace;

	
	// Handlers
	void onNewImg();

};

} //: namespace RodsDetection
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RodsDetection", Processors::RodsDetection::RodsDetection)

#endif /* RODSDETECTION_HPP_ */
