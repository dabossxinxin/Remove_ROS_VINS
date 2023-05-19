cmake_minimum_required(VERSION 3.18)
project(vins)

cmake_policy(SET CMP0111 OLD)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
if(WIN32)
	add_definitions(-DNOMINMAX)
	set(CMAKE_CXX_FLAGS_DEBUG "/bigobj /EHsc")
	set(CMAKE_CXX_FLAGS_RELEASE "-O2 /bigobj /EHsc")
endif()

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/../include)
include_directories(${CMAKE_SOURCE_DIR}/../camera_model/include)


########################################################
#													   #
#				  编译camera model库			       #
#													   #	
########################################################
file(GLOB CalibHdrs			"../camera_model/include/camodocal/calib/*.h")
file(GLOB CameraHdrs		"../camera_model/include/camodocal/camera_models/*.h")
file(GLOB ChessBoardHdrs	"../camera_model/include/camodocal/chessboard/*.h")
file(GLOB GplHdrs			"../camera_model/include/camodocal/gpl/*.h")
file(GLOB SparseGraphHdrs	"../camera_model/include/camodocal/sparse_graph/*.h")

file(GLOB CalibSrcs			"../camera_model/src/calib/*.cc")
file(GLOB CameraSrcs		"../camera_model/src/camera_models/*.cc")
file(GLOB ChessBoardSrcs	"../camera_model/src/chessboard/*.cc")
file(GLOB GplSrcs			"../camera_model/src/gpl/*.cc")
file(GLOB SparseGraphSrcs	"../camera_model/src/sparse_graph/*.cc")

source_group("calib src"		FILES ${CalibSrcs})
source_group("calib hdr"		FILES ${CalibHdrs})
source_group("camera src"		FILES ${CameraSrcs})
source_group("camera hdr"		FILES ${CameraHdrs})
source_group("chess_board src"	FILES ${ChessBoardSrcs})
source_group("chess_board hdr"	FILES ${ChessBoardHdrs})
source_group("gpl src"			FILES ${GplSrcs})
source_group("gpl hdr"			FILES ${GplHdrs})
source_group("sparse_graph src" FILES ${SparseGraphSrcs})
source_group("sparse_graph hdr" FILES ${SparseGraphHdrs})

add_library(camera_model
    ${CalibSrcs}
	${CalibHdrs}
	${CameraSrcs}
	${CameraHdrs}
	${ChessBoardSrcs}
	${ChessBoardHdrs}
	${GplSrcs}
	${GplHdrs}
	${SparseGraphSrcs}
	${SparseGraphHdrs}
)

target_link_libraries(camera_model 
	${CERES_LIBRARIES} 
	${OpenCV_LIBRARIES}
)

########################################################
#													   #
#			          编译DBoW库			           #
#													   #	
########################################################
file(GLOB VocaHdrs		"./src/loop_closure/ThirdParty/*.hpp")
file(GLOB DBoWHdrs		"./src/loop_closure/ThirdParty/DBoW/*.h")
file(GLOB DUtilsHdrs	"./src/loop_closure/ThirdParty/DUtils/*.h")
file(GLOB DVsionHdrs	"./src/loop_closure/ThirdParty/DVision/*.h")

file(GLOB VocaSrcs		"./src/loop_closure/ThirdParty/*.cpp")
file(GLOB DBoWSrcs		"./src/loop_closure/ThirdParty/DBoW/*.cpp")
file(GLOB DUtilsSrcs	"./src/loop_closure/ThirdParty/DUtils/*.cpp")
file(GLOB DVsionSrcs	"./src/loop_closure/ThirdParty/DVision/*.cpp")

source_group("voc hdr"			FILES ${VocaHdrs})
source_group("voc src"			FILES ${VocaSrcs})
source_group("DBoW hdr"			FILES ${DBoWHdrs})
source_group("DBoW src"			FILES ${DBoWSrcs})
source_group("DUtils hdr" 		FILES ${DUtilsHdrs})
source_group("DUtils src" 		FILES ${DUtilsSrcs})
source_group("DVsion hdr" 		FILES ${DVsionHdrs})
source_group("DVsion src" 		FILES ${DVsionSrcs})

add_library(DBoW 
	${VocaHdrs}
	${VocaSrcs}
	${DBoWHdrs}
	${DBoWSrcs}
	${DUtilsHdrs}
	${DUtilsSrcs}
	${DVsionHdrs}
	${DVsionSrcs}
)

target_link_libraries(DBoW 
	${OpenCV_LIBRARIES}
) 

########################################################
#													   #
#			编译visual inertial odometry库			   #
#													   #	
########################################################
file(GLOB VioHdrs				"./src/*.h")
file(GLOB VioFactorHdrs			"./src/factor/*.h")
file(GLOB VioUtilityHdrs		"./src/utility/*.h")
file(GLOB VioInitialHdrs		"./src/initial/*.h")
file(GLOB VioLoopClosureHdrs	"./src/loop_closure/*.h")
file(GLOB VioFeatureTrackHdrs	"./src/feature_tracker/*.h")

file(GLOB VioSrcs				"./src/*.cpp")
file(GLOB VioFactorSrcs			"./src/factor/*.cpp")
file(GLOB VioUtilitySrcs		"./src/utility/*.cpp")
file(GLOB VioInitialSrcs		"./src/initial/*.cpp")
file(GLOB VioLoopClosureSrcs	"./src/loop_closure/*.cpp")
file(GLOB VioFeatureTrackSrcs	"./src/feature_tracker/*.cpp")

source_group("main src"				FILES ${VioSrcs})
source_group("main hdr"				FILES ${VioHdrs})
source_group("factor src"			FILES ${VioFactorSrcs})
source_group("factor hdr"			FILES ${VioFactorHdrs})
source_group("utility src" 			FILES ${VioUtilitySrcs})
source_group("utility hdr" 			FILES ${VioUtilityHdrs})
source_group("initial src" 			FILES ${VioInitialSrcs})
source_group("initial hdr" 			FILES ${VioInitialHdrs})
source_group("loop_closure src" 	FILES ${VioLoopClosureSrcs})
source_group("loop_closure hdr" 	FILES ${VioLoopClosureHdrs})
source_group("feature_tracker src" 	FILES ${VioFeatureTrackSrcs})
source_group("feature_tracker hdr" 	FILES ${VioFeatureTrackHdrs})

add_library(vins_estimator 
	${VioHdrs} 
	${VioSrcs}
	${VioFactorSrcs}
	${VioFactorHdrs}
	${VioUtilitySrcs}
	${VioUtilityHdrs}
	${VioInitialSrcs}
	${VioInitialHdrs}
	${VioLoopClosureSrcs}
	${VioLoopClosureHdrs}
	${VioFeatureTrackSrcs}
	${VioFeatureTrackHdrs}
)

target_link_libraries(vins_estimator 
	${CERES_LIBRARIES} 
	${Pangolin_LIBRARIES} 
	${OpenCV_LIBRARIES}
	camera_model
	DBoW
) 

add_executable(example ./src/example/estimator_node.cpp)
target_link_libraries(example vins_estimator)

add_executable(example_vis ./src/example/estimator_visualization.cpp)
target_link_libraries(example_vis vins_estimator)

add_executable(example_without_loop ./src/example/estimator_without_loop.cpp)
target_link_libraries(example_without_loop vins_estimator)