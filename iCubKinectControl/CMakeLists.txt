# Copyright (C) 2014 CoDyCo
# Author: Daniele Pucci, Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME iCubKinectControl)
PROJECT(${PROJECTNAME})

find_package(YARP)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
SET(KinectSDK_ROOT_DIR $ENV{KINECTSDK20_DIR})

set(SOURCES 
    src/BodyBasics.cpp
	src/YarpAdapter.cpp
	src/Vector.cpp
    )

SOURCE_GROUP("Source Files" FILES ${SOURCES})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(
${YARP_INCLUDE_DIRS}
${CMAKE_SOURCE_DIR}/include

${KinectSDK_ROOT_DIR}/inc
${KinectSDK_ROOT_DIR}/bin/x86
)

MESSAGE(${KinectSDK_ROOT_DIR})
ADD_EXECUTABLE(${PROJECTNAME} ${SOURCES})
add_definitions(-DUNICODE -D_UNICODE)
TARGET_LINK_LIBRARIES(${PROJECTNAME}
 ${YARP_LIBRARIES})

 SET(LIB1 ${KinectSDK_ROOT_DIR}Lib/x86/Kinect20.Face.lib)
 SET(LIB2 ${KinectSDK_ROOT_DIR}Lib/x86/Kinect20.Fusion.lib)
 SET(LIB3 ${KinectSDK_ROOT_DIR}Lib/x86/Kinect20.lib)
 SET(LIB4 ${KinectSDK_ROOT_DIR}Lib/x86/Kinect20.VisualGestureBuilder.lib)
 add_definitions(-DUNICODE -D_UNICODE)

IF(WIN32)
		target_link_libraries(${PROJECTNAME} ${LIB1} ${LIB2} ${LIB3} ${LIB4})
        INSTALL_TARGETS(/bin/Release ${PROJECTNAME})    
		if(MSVC) # Check if we are using the Visual Studio compiler
			set_target_properties(${PROJECTNAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
			SET_TARGET_PROPERTIES (${PROJECTNAME} PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:windows")
		endif()
else()
        INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)
