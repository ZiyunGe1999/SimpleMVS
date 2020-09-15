#include<iostream>
#include"jsonReader.hpp"

int main(int argc, char** argv){

    if(argc != 2){
        std::cout << "Usage: ./bin/SimpleMVS <PATH/TO/CONFIG/FILE>" << std::endl;
        return 0;
    }

    std::string pathToConfig = argv[1];
    std::cout << "Loading File at " << pathToConfig << std::endl;
    jsonFileReader configFileReader(pathToConfig);
    std::cout << "Loaded JSON File Successfully!!!" << std::endl;

    std::cout << "The Image Directory is at " << configFileReader.getImageDirPath() << std::endl;
    std::cout << "The Pose File is at " << configFileReader.getPoseFilePath() << std::endl;
    std::cout << "The Image Directory is at " << configFileReader.getCameraCalibrationFilePath() << std::endl;

    return 0;
}