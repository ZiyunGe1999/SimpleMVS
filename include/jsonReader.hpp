#ifndef JSONREADER_HPP
#define JSONREADER_HPP

#include "json.h"
#include<iostream>
#include <string>
#include <fstream>
#include <glog/logging.h>
#include <parameters.hpp>

class jsonFileReader{
    private:
        std::string pathToConfig;
        Json::Value root;
        
    
    public:
        jsonFileReader(){};
        jsonFileReader(std::string givenPath);
        ~jsonFileReader(){};
        void setPathToConfig(std::string givenPath);
        std::string getImageDirPath();
        std::string getPoseFilePath();
        std::string getCameraCalibrationFilePath();
        Json::Value JsonParser(const std::string fileName);
        SetParameters getUserSetParameters();

};

#endif