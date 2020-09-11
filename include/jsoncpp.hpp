#include "../Thirdparty/jsoncpp/dist/json/json.h"
// #include "../Thirdparty/jsoncpp/dist/jsoncpp.cpp"
#include <string>

class jsonFileReader{
    private:
        std::string pathToConfig;
        jsonFileReader(){};
        jsonFileReader(std::string givenPath);
        ~jsonFileReader(){};
    
    public:
        void setPathToConfig(std::string givenPath);
        std::string getImageDirPath();
        std::string getPoseFilePath();
        std::string getCameraCalibrationFilePath();

};