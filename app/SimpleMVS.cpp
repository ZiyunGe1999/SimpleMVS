#include <iostream>
#include "jsonReader.hpp"
#include "images.hpp"
#include "cameras.hpp"
#include <glog/logging.h>
#include "MVSProcess.hpp"

int main(int argc, char** argv){

    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);

    if(argc != 2){
        LOG(FATAL) << "Usage: ./bin/SimpleMVS <PATH/TO/CONFIG/FILE>";
        // std::cout << "Usage: ./bin/SimpleMVS <PATH/TO/CONFIG/FILE>" << std::endl;
        return 0;
    }

    std::string path_to_Config = argv[1];
    LOG(INFO) << "Loading File at " << path_to_Config;
    jsonFileReader config_file_reader(path_to_Config);
    LOG(INFO) << "Loaded JSON File Successfully!!!";

    LOG(INFO) << "The Image Directory is at " << config_file_reader.getImageDirPath();
    LOG(INFO) << "The Pose File is at " << config_file_reader.getPoseFilePath();
    LOG(INFO) << "The Camera Calibration File is at " << config_file_reader.getCameraCalibrationFilePath();

    //Get User Set Parameters
    LOG(INFO) << "Get User Set Parameters";
    SetParameters set_parameters = config_file_reader.getUserSetParameters();

    // Load cameras (indexed by: camera_id).
    ColmapCameraPtrMap cameras;
    std::string cameras_txt_path = config_file_reader.getCameraCalibrationFilePath();
    bool success = ReadColmapCameras(cameras_txt_path, &cameras);
    if (success) {
    LOG(INFO) << "Successfully loaded " << cameras.size() << " camera(s).";
    } else {
    LOG(FATAL) << "Error: could not load cameras." << std::endl;
    }

    // Load images (indexed by: image_id).
    ColmapImagePtrMap images;
    std::string images_txt_path = config_file_reader.getPoseFilePath();
    success = ReadColmapImages(images_txt_path, /* read_observations */ true,
                                &images);
    if (success) {
    LOG(INFO) << "Successfully loaded " << images.size() << " image info(s)." << std::endl;
    } else {
    LOG(FATAL) << "Error: could not load image infos." << std::endl;
    }

    //MVS Main Process
    MVSProcess simple_mvs_process(config_file_reader.getImageDirPath(), set_parameters, cameras);
    for(auto iter = images.begin(); iter != images.end(); iter++){
        LOG(INFO) << "=================================================================================";
        LOG(INFO) << "Update frames for simple mvs process";
        LOG(INFO) << "Insert frame " << iter->first << " into the process";
        simple_mvs_process.updateFrames(iter->first, images);

        LOG(INFO) << "---------------------------------------------------------------------------------";
        simple_mvs_process.initializePlanes();
    }

    return 0;
}