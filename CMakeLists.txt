CMAKE_MINIMUM_REQUIRED(VERSION 2.7)

project(SimpleMVS)

SET(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

find_package(glog 0.4.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "======================================================================")
message(STATUS "----------------Eigen is at " ${EIGEN3_INCLUDE_DIRS})
message(STATUS "---------------Opencv is at " ${OpenCV_INCLUDE_DIRS})
# message(STATUS "glog is at " ${GLOG_LIB_DIRS})

set(JSON_LIB_PATH Thirdparty/jsoncpp/dist)

add_library(json SHARED
    ${JSON_LIB_PATH}/jsoncpp.cpp
)

include_directories(
    include
    ${JSON_LIB_PATH}
    ${JSON_LIB_PATH}/json
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# -----read Json file-------------------------
add_library(readJsonFile SHARED
    src/readJsonFile.cpp
)
target_link_libraries(readJsonFile
    json
)
# --------------------------------------------

# -----COLMAP text format loader--------------
add_library(COLMAPFormatLoader SHARED
    src/cameras.cpp
    src/images.cpp
)
# --------------------------------------------

# -----MVSProcess | core of the simpleMVS ----
add_library(MVSProcess SHARED
    src/MVSProcess.cpp
)
target_link_libraries(MVSProcess
    ${OpenCV_LIBS}
    glog::glog
)
# --------------------------------------------


# -----App------------------------------------
add_executable(SimpleMVS_run 
    app/SimpleMVS.cpp 
    # src/readJsonFile.cpp
)
target_link_libraries(SimpleMVS_run
    readJsonFile
    COLMAPFormatLoader
    glog::glog
    MVSProcess
)
# --------------------------------------------