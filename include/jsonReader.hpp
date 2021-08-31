#ifndef JSONREADER_HPP
#define JSONREADER_HPP

#include "json.h"
#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <parameters.hpp>
#include <string>

class jsonFileReader {
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
    std::string getOutputDirectoryPath();
    Json::Value JsonParser(const std::string fileName);
    SetParameters getUserSetParameters();
};

#endif