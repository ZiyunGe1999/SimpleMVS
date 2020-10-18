#include<iostream>
#include"jsonReader.hpp"
#include"images.hpp"
#include"cameras.hpp"
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
    LOG(INFO) << "The Camera Calibration File is at " << configFileReader.getCameraCalibrationFilePath();

    // Load cameras (indexed by: camera_id).
    ColmapCameraPtrMap cameras;
    std::string cameras_txt_path = configFileReader.getCameraCalibrationFilePath();
    bool success = ReadColmapCameras(cameras_txt_path, &cameras);
    if (success) {
    LOG(INFO) << "Successfully loaded " << cameras.size() << " camera(s).";
    } else {
    LOG(FATAL) << "Error: could not load cameras." << std::endl;
    }

    // Load images (indexed by: image_id).
    ColmapImagePtrMap images;
    std::string images_txt_path = configFileReader.getPoseFilePath();
    success = ReadColmapImages(images_txt_path, /* read_observations */ true,
                                &images);
    if (success) {
    LOG(INFO) << "Successfully loaded " << images.size() << " image info(s)." << std::endl;
    } else {
    LOG(FATAL) << "Error: could not load image infos." << std::endl;
    }

    return 0;
}