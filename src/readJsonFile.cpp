#include "jsonReader.hpp"

jsonFileReader::jsonFileReader(std::string givenPath) : pathToConfig(givenPath) { root = JsonParser(givenPath); }

Json::Value jsonFileReader::JsonParser(const std::string fileName) {
    Json::CharReaderBuilder Parser;
    Json::Value Root;
    Json::String errors;

    Parser["collectComments"] = false;

    std::ifstream InFile(fileName, std::ios::in);
    if (InFile) {
        // Parser.parse(InFile, Root);
        Json::parseFromStream(Parser, InFile, &Root, &errors);
        InFile.close();
    } else {
        InFile.close();
        LOG(FATAL) << "ERROR: didn't find file at " << fileName;
    }

    return Root;
}

void jsonFileReader::setPathToConfig(std::string givenPath) { pathToConfig = givenPath; }

std::string jsonFileReader::getImageDirPath() {
    std::string imageDirPath = root["ImageDir"].asString();
    return imageDirPath;
}

std::string jsonFileReader::getPoseFilePath() {
    std::string poseFilePath = root["PoseFile"].asString();
    return poseFilePath;
}

std::string jsonFileReader::getCameraCalibrationFilePath() {
    std::string cameraCalibrationFilePath = root["CamCalibration"].asString();
    return cameraCalibrationFilePath;
}

std::string jsonFileReader::getOutputDirectoryPath() {
    std::string output_directory = root["OutputDirectory"].asString();
    return output_directory;
}

SetParameters jsonFileReader::getUserSetParameters() {
    SetParameters user_set_parameters;
    user_set_parameters.max_z_ = root["parameters"]["max_z"].asDouble();
    user_set_parameters.min_z_ = root["parameters"]["min_z"].asDouble();
    user_set_parameters.zoom = root["parameters"]["zoom"].asFloat();
    user_set_parameters.patch_length = root["parameters"]["patch_length"].asInt();
    user_set_parameters.patch_step = root["parameters"]["patch_step"].asInt();
    return user_set_parameters;
}