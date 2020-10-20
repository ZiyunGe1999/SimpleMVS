#include"MVSProcess.hpp"

MVSFrame::MVSFrame(int id, std::string imageRootPath, ColmapImagePtrMap &map) : image_id(id) {
    ColmapImagePtr imagePtr = map[image_id];
    camera_id = imagePtr->camera_id;
    image_T_global = imagePtr->image_T_global;
    global_T_image = imagePtr->global_T_image;

    std::string imageFile = imagePtr->file_path;
    pixels = cv::imread(imageRootPath + "/" + imageFile, cv::IMREAD_COLOR);
    if(pixels.empty()){
        LOG(FATAL) << "Could not read the image: " << imageRootPath;
    }

    //initialize planes whose size is the same as pixels' size
    planes.resize(pixels.rows);
    for(int i=0; i<pixels.rows; i++){
        planes[i].resize(pixels.cols);
    }

}