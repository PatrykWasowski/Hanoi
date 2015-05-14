/*!
 * \file
 * \brief 
 * \author Patryk Wasowski
 */

#ifndef FINDPIECE_HPP_
#define FINDPIECE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"


namespace Processors {
namespace FindPiece {

/*!
 * \class FindPiece
 * \brief FindPiece processor class.
 *
 * 
 */
class FindPiece: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	FindPiece(const std::string & name = "FindPiece");

	/*!
	 * Destructor
	 */
	virtual ~FindPiece();

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
    ros::NodeHandle * nh;

	// Properties
	Base::Property<std::string> ros_topic_name;
	Base::Property<std::string> ros_namespace;

	
	// Handlers
	void onNewImg();

};

} //: namespace FindPiece
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("FindPiece", Processors::FindPiece::FindPiece)

#endif /* FINDPIECE_HPP_ */
