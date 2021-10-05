#include "MVSProcess.hpp"
#include "cameras.hpp"
#include "images.hpp"
#include "jsonReader.hpp"
#include <glog/logging.h>
#include <iostream>

int main(int argc, char **argv) {

    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);

    if (argc != 2) {
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

    // Get User Set Parameters
    LOG(INFO) << "Get User Set Parameters";
    SetParameters set_parameters = config_file_reader.getUserSetParameters();

    // Load cameras (indexed by: camera_id).
    ColmapCameraPtrMap cameras;
    std::string cameras_txt_path = config_file_reader.getCameraCalibrationFilePath();
    bool success = ReadColmapCameras(cameras_txt_path, &cameras);
    if (success) {
        LOG(INFO) << "Successfully loaded " << cameras.size() << " camera(s).";
    } else {
        LOG(FATAL) << "Error: could not load cameras.";
    }

    // Load images (indexed by: image_id).
    ColmapImagePtrMap images;
    std::string images_txt_path = config_file_reader.getPoseFilePath();
    success = ReadColmapImages(images_txt_path, /* read_observations */ true, images);
    if (success) {
        LOG(INFO) << "Successfully loaded " << images.size() << " image info(s).";
    } else {
        LOG(FATAL) << "Error: could not load image infos.";
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pose_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto &image : images) {
        // LOG(INFO) << image.first;
        Eigen::Vector3f position = image.second->global_T_image.translation();
        // LOG(INFO) << position.transpose();
        pcl::PointXYZRGB p;
        p.x = position[0];
        p.y = position[1];
        p.z = position[2];
        p.r = 0;
        p.g = 0;
        p.b = 0;
        pose_cloud->push_back(p);
    }
    pcl::io::savePCDFileASCII(config_file_reader.getOutputDirectoryPath() + "/pose_cloud.pcd", *pose_cloud);
    // exit(0);

    // MVS Main Process
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    MVSProcess simple_mvs_process(config_file_reader.getImageDirPath(), set_parameters, cameras, cloud);
    for (auto iter = images.begin(); iter != images.end(); iter++) {
        LOG(INFO) << "=================================================================================";
        LOG(INFO) << "Update frames for simple mvs process";
        LOG(INFO) << "Insert frame " << iter->first << " into the process";
        simple_mvs_process.updateFrames(iter->first, images);

        LOG(INFO) << "---------------------------------------------------------------------------------";
        simple_mvs_process.initializePlanes();

        LOG(INFO) << "---------------------------------------------------------------------------------";
        simple_mvs_process.planePropagation();

        LOG(INFO) << "---------------------------------------------------------------------------------";
        simple_mvs_process.checkConsistency();

        // if (iter->first >= 3) {
        //     break;
        // }
    }

    std::string pcd_filename = config_file_reader.getOutputDirectoryPath() + "/mvs_result.pcd";
    if (cloud->empty()) {
        LOG(INFO) << "There is no point in the cloud";
    } else {
        LOG(INFO) << "Saving point cloud at " << pcd_filename;
        pcl::io::savePCDFileASCII(pcd_filename, *cloud);
    }

    return 0;
}