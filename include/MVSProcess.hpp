#ifndef MVSPROCESS_HPP
#define MVSPROCESS_HPP

#include<iostream>
#include<string>
#include"images.hpp"
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

struct MVSPlane{
    float x = 0;
    float y = 0;
    float z = 0;
    float w = 0;
};

class MVSFrame{
    private:
        // std::string imageRootPath;
        // std::vector<std::vector<u_char> > pixels;
        int image_id;
        int camera_id;
        cv::Mat pixels;
        std::vector<std::vector<MVSPlane> > planes;
        SE3f image_T_global;
        SE3f global_T_image;
    public:
        MVSFrame(int id, std::string path, ColmapImagePtrMap &map);

        
};

class MVSProcess{

};


#endif