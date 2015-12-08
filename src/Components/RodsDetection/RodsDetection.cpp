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

RodsDetection::RodsDetection(const std::string & name) :
		Base::Component(name) , 
		ros_topic_name("ros_topic_name", rods), 
		ros_namespace("ros_namespace", discode) {
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

bool RodsDetection::onInit() {

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
}



} //: namespace RodsDetection
} //: namespace Processors
