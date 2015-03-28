cmake_minimum_required(VERSION 3.0)

project(KinectV2Imaq CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    MESSAGE( "64 bit compiler detected" )
    SET( EX_PLATFORM 64 )
    SET( EX_BIT_COUNT 64)
    SET( EX_PLATFORM_NAME "x64" )
else( CMAKE_SIZEOF_VOID_P EQUAL 8 ) 
    MESSAGE( "32 bit compiler detected" )
    SET( EX_PLATFORM 32 )
    SET( EX_BIT_COUNT 32)
    SET( EX_PLATFORM_NAME "x86" )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
find_package(Matlab REQUIRED)

set(KINECTV2_INCLUDE_DIR $ENV{KINECTSDK20_DIR}inc)
set(KINECTV2_LIBRARIES $ENV{KINECTSDK20_DIR}Lib/${EX_PLATFORM_NAME}/Kinect20.lib)

include_directories(${KINECTV2_INCLUDE_DIR} ${MATLAB_ROOT}/toolbox/imaq/imaqadaptors/kit/include)

set(MATLAB_IMAQ_LIBRARY ${MATLAB_ROOT}/toolbox/imaq/imaqadaptors/kit/lib/win${EX_BIT_COUNT}/release/imaqmex.lib)

set(KV2I_INCLUDE_DIR "include")
set(KV2I_SOURCE_DIR "src")

set(KV2I_INCLUDE_FILES
		${KV2I_INCLUDE_DIR}/ColourAdapter.h
		${KV2I_INCLUDE_DIR}/DepthAdapter.h
		${KV2I_INCLUDE_DIR}/InfraredAdapter.h
		${KV2I_INCLUDE_DIR}/LongExposureInfraredAdapter.h
		${KV2I_INCLUDE_DIR}/KinectDeviceInfo.h)

set(KV2I_SOURCE_FILES
		${KV2I_SOURCE_DIR}/ColourAdapter.cpp
		${KV2I_SOURCE_DIR}/DepthAdapter.cpp
		${KV2I_SOURCE_DIR}/InfraredAdapter.cpp
		${KV2I_SOURCE_DIR}/LongExposureInfraredAdapter.cpp
		${KV2I_SOURCE_DIR}/KinectDeviceInfo.cpp
		${KV2I_SOURCE_DIR}/KinectV2Imaq_export.cpp)

add_library(KinectV2Imaq SHARED ${KV2I_SOURCE_FILES} ${KV2I_INCLUDE_FILES})
target_link_libraries(KinectV2Imaq ${MATLAB_IMAQ_LIBRARY} ${KINECTV2_LIBRARIES})