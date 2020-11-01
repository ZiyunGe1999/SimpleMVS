#ifndef MVSPROCESS_HPP
#define MVSPROCESS_HPP

#include <iostream>
#include <string>
#include "images.hpp"
#include "cameras.hpp"
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <memory>
#include "parameters.hpp"
#include <random>
#include <math.h>

struct MVSPlane{
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

class MVSFrame{
    public:
        // std::string imageRootPath;
        // std::vector<std::vector<u_char> > pixels;
        int image_id;
        int camera_id;
        cv::Mat pixels;
        std::vector<std::vector<MVSPlane> > planes;
        SE3f image_T_global;
        SE3f global_T_image;
    public:
        MVSFrame(int id, std::string image_root_path, ColmapImagePtrMap &map);

        
};

class MVSProcess{
    private:
        SetParameters set_parameters_;
        // ColmapCameraPtrMap cameras_;
        std::unordered_map<int, std::vector<std::vector<Eigen::Vector3d> > > view_vectors_;
        std::string image_root_path_;
        // ColmapImagePtrMap map_;
        std::shared_ptr<MVSFrame> frame_ptr_1, frame_ptr_2, frame_ptr_3;
    public:
        MVSProcess(std::string image_root_path, SetParameters set_parameters, ColmapCameraPtrMap &cameras);
        ~MVSProcess();
        Eigen::Vector3d getViewVector(int camera_id, int row, int col);
        void updateFrames(int id, ColmapImagePtrMap &map);
        void initializePlanes();
};


#endif