#ifndef MVSPROCESS_HPP
#define MVSPROCESS_HPP

#include "cameras.hpp"
#include "images.hpp"
#include "parameters.hpp"
#include <algorithm>
#include <ctime>
#include <glog/logging.h>
#include <iostream>
#include <math.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define NON_OBSERVABLE_COST (FLT_MAX)

struct MVSPlane {
    float x = 0;
    float y = 0;
    float z = 0;
    float w = 0;

    Eigen::Vector3d vector() { return Eigen::Vector3d(x, y, z); };
    Eigen::Vector3d getNormalizedVector() { return Eigen::Vector3d(x, y, z).normalized(); };
};

class MVSFrame {
  public:
    // std::string imageRootPath;
    // std::vector<std::vector<u_char> > pixels;
    int image_id;
    int camera_id;
    cv::Mat pixels;
    cv::Mat pixels_color;
    std::vector<std::vector<MVSPlane>> planes;
    SE3f image_T_global;
    SE3f global_T_image;

    int rows() { return int(pixels.rows); };
    int cols() { return int(pixels.cols); };

  public:
    MVSFrame(int id, std::string image_root_path, ColmapImagePtrMap &map, float zoom = 1);
    MVSFrame(){};
    // ~MVSFrame(){LOG(INFO) << "Deconstruct frame " << image_id;};
};

class MVSProcess {
  private:
    SetParameters set_parameters_;
    ColmapCameraPtrMap cameras_;
    std::unordered_map<int, std::vector<std::vector<Eigen::Vector3d>>> view_vectors_;
    std::string image_root_path_;
    // ColmapImagePtrMap map_;
    std::shared_ptr<MVSFrame> frame_ptr_1, frame_ptr_2, frame_ptr_3;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  public:
    MVSProcess(std::string image_root_path, SetParameters set_parameters, ColmapCameraPtrMap &cameras,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
    ~MVSProcess();
    Eigen::Vector3d getViewVector(int camera_id, int row, int col);
    void updateFrames(int id, ColmapImagePtrMap &map);
    void initializePlanes();
    void planePropagation();
    void checkConsistency();
    float calculateProjectionCost(Eigen::Vector3d view_vector, MVSPlane plane, float val_original,
                                  float grad_horizon_original, float grad_vertical_original);
    bool projectToCamera(Eigen::Vector3f global_point, std::shared_ptr<MVSFrame> frame_ptr, size_t &pixel_x,
                         size_t &pixel_y);
    void saveAsDepthImage(std::shared_ptr<MVSFrame> frame_ptr, std::string output_filename);
    float calculatePlaneProjectionCost(int i, int j, MVSPlane plane);
    void refinePlane(size_t i, size_t j, float cost);
    void propagatePlanesFromPreviousFrame();
};

#endif