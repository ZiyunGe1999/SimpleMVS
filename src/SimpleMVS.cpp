#include<iostream>
#include"jsonReader.hpp"
#include <glog/logging.h>

int main(int argc, char** argv){

    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);

    if(argc != 2){
        LOG(FATAL) << "Usage: ./bin/SimpleMVS <PATH/TO/CONFIG/FILE>";
        // std::cout << "Usage: ./bin/SimpleMVS <PATH/TO/CONFIG/FILE>" << std::endl;
        return 0;
    }

    std::string pathToConfig = argv[1];
    LOG(INFO) << "Loading File at " << pathToConfig;
    jsonFileReader configFileReader(pathToConfig);
    LOG(INFO) << "Loaded JSON File Successfully!!!";

    LOG(INFO) << "The Image Directory is at " << configFileReader.getImageDirPath();
    LOG(INFO) << "The Pose File is at " << configFileReader.getPoseFilePath();
    LOG(INFO) << "The Image Directory is at " << configFileReader.getCameraCalibrationFilePath();

    return 0;
}