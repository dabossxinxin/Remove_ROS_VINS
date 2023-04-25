cmake_minimum_required(VERSION 3.14)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(_WIN_)
	set(CMAKE_CXX_FLAGS_DEBUG "/bigobj /EHsc")
	set(CMAKE_CXX_FLAGS_RELEASE "-O2 /bigobj /EHsc")
endif()

include_directories(${CMAKE_SOURCE_DIR}/../camera_model/include)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

#find_path(OpenCV_INCLUDE_DIRS NAMES opencv2 HINTS ${CMAKE_SOURCE_DIR}/../../SDK/opencv-3.4.6/include)
#find_file(OpenCV_DEBUG_LIB opencv_world346d.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/opencv-3.4.6/lib)
#find_file(OpenCV_RELEASE_LIB opencv_world346.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/opencv-3.4.6/lib)

#find_path(GLEW_INCLUDE_DIRS NAMES GL HINTS ${CMAKE_SOURCE_DIR}/../../SDK/glew/include)
#find_file(GLEW_DEBUG_LIB glewd.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/glew/lib)
#find_file(GLEW_RELEASE_LIB glew.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/glew/lib)

#find_path(Pangolin_INCLUDE_DIRS pangolin HINTS ${CMAKE_SOURCE_DIR}/../../Pangolin/include)
#find_path(Pangolin_LIB_DIR pangolin.lib HINTS ${CMAKE_SOURCE_DIR}/../../Pangolin/lib)
#set(Pangolin_DEBUG_LIB ${Pangolin_LIB_DIR}/pangolind.lib)
#set(Pangolin_RELEASE_LIB ${Pangolin_LIB_DIR}/pangolin.lib)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(vins_estimator
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    #src/utility/CameraPoseVisualization.cpp
	src/utility/print.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/loop-closure/ThirdParty/DBoW/BowVector.cpp
    src/loop-closure/ThirdParty/DBoW/FBrief.cpp
    src/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
    src/loop-closure/ThirdParty/DBoW/QueryResults.cpp
    src/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
    src/loop-closure/ThirdParty/DUtils/Random.cpp
    src/loop-closure/ThirdParty/DUtils/Timestamp.cpp
    src/loop-closure/ThirdParty/DVision/BRIEF.cpp
    src/loop-closure/ThirdParty/VocabularyBinary.cpp
    src/loop-closure/loop_closure.cpp
    src/loop-closure/keyframe.cpp
    src/loop-closure/keyframe_database.cpp
    src/feature_tracker/feature_tracker.cpp 
    #src/feature_tracker/feature_tracker_parameters.cpp
    ../camera_model/src/chessboard/Chessboard.cc
    ../camera_model/src/calib/CameraCalibration.cc
    ../camera_model/src/camera_models/Camera.cc
    ../camera_model/src/camera_models/CameraFactory.cc
    ../camera_model/src/camera_models/CostFunctionFactory.cc
    ../camera_model/src/camera_models/PinholeCamera.cc
    ../camera_model/src/camera_models/CataCamera.cc
    ../camera_model/src/camera_models/EquidistantCamera.cc
    ../camera_model/src/camera_models/ScaramuzzaCamera.cc
    ../camera_model/src/sparse_graph/Transform.cc
    ../camera_model/src/gpl/gpl.cc
    ../camera_model/src/gpl/EigenQuaternionParameterization.cc
    )

target_link_libraries(vins_estimator 
	${CERES_LIBRARIES} 
	${Pangolin_LIBRARIES} 
	${OpenCV_LIBRARIES}
) 



