# Copyright (C) 2014 CoDyCo
# Author: Daniele Pucci, Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME iCubOculusControl)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

set(SOURCES 
    src/ICubOculusController.cpp
	src/OculusAdapter.cpp
	src/YarpAdapter.cpp
	src/MultidimensionalFilter.cpp
	src/Filter.cpp
)

SOURCE_GROUP("Source Files" FILES ${SOURCES})

 include(iCubHelpers)
 include(iCubOptions)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(
${YARP_INCLUDE_DIRS}
${CMAKE_SOURCE_DIR}/include
"D:/Uni/Learning Robots/IP/LibOVR/Src"
"D:/Uni/Learning Robots/IP/LibOVR/Include"
)
MESSAGE(${YARP_LIBRARIES})

link_directories("D:/Uni/Learning Robots/IP/LibOVR/Lib/Win32/VS2013")

ADD_EXECUTABLE(${PROJECTNAME} ${SOURCES})
add_definitions(-DUNICODE -D_UNICODE)
TARGET_LINK_LIBRARIES(${PROJECTNAME}
 ${YARP_LIBRARIES}  
)
IF(WIN32)
		target_link_libraries(${PROJECTNAME})
        INSTALL_TARGETS(/bin/Release ${PROJECTNAME})    
		if(MSVC) # Check if we are using the Visual Studio compiler
			set_target_properties(${PROJECTNAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
			SET_TARGET_PROPERTIES (${PROJECTNAME} PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:windows")
		endif()
else()
        INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)
