#include "../include/jsoncpp.hpp"

jsonFileReader::jsonFileReader(std::string givenPath): pathToConfig(givenPath)
{

}

void jsonFileReader::setPathToConfig(std::string givenPath){
    pathToConfig = givenPath;
}

std::string jsonFileReader::getImageDirPath(){
    std::string imageDirPath;
    return imageDirPath;
}

std::string jsonFileReader::getPoseFilePath(){
    std::string poseFilePath;
    return poseFilePath;
}

std::string jsonFileReader::getCameraCalibrationFilePath(){
    std::string cameraCalibrationFilePath;
    return cameraCalibrationFilePath;
}