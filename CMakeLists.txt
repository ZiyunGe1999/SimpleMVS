CMAKE_MINIMUM_REQUIRED(VERSION 2.7)

project(SimpleMVS)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(JSON_LIB_PATH Thirdparty/jsoncpp/dist)

add_library(json SHARED
    ${JSON_LIB_PATH}/jsoncpp.cpp
)

include_directories(
    include
    ${JSON_LIB_PATH}
    ${JSON_LIB_PATH}/json
)

ADD_EXECUTABLE(SimpleMVS_run 
    src/SimpleMVS.cpp 
    src/readJsonFile.cpp)

target_link_libraries(SimpleMVS_run
    json
)