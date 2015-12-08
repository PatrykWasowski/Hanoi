# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

USE_ROS(roscpp std_msgs)

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )


# Create an executable file from sources:
ADD_LIBRARY(RedDiskDetection SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(RedDiskDetection ${DisCODe_LIBRARIES} ${ROS_LIBRARIES}
	${OpenCV_LIBS})

INSTALL_COMPONENT(RedDiskDetection)
