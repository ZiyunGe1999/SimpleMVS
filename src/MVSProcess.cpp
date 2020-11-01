#include"MVSProcess.hpp"

MVSFrame::MVSFrame(int id, std::string image_root_path, ColmapImagePtrMap &map) : image_id(id) {
    ColmapImagePtr image_ptr = map[image_id];
    camera_id = image_ptr->camera_id;
    image_T_global = image_ptr->image_T_global;
    global_T_image = image_ptr->global_T_image;

    std::string image_file_path = image_ptr->file_path;
    pixels = cv::imread(image_root_path + "/" + image_file_path, cv::IMREAD_COLOR);
    if(pixels.empty()){
        LOG(FATAL) << "Could not read the image: " << image_root_path;
    }

    //initialize planes whose size is the same as pixels' size
    planes.resize(pixels.rows);
    for(int i=0; i<pixels.rows; i++){
        planes[i].resize(pixels.cols);
    }

}

MVSProcess::MVSProcess(std::string image_root_path, SetParameters set_parameters, ColmapCameraPtrMap &cameras)
             : image_root_path_(image_root_path), set_parameters_(set_parameters){
    // view_vectors_.resize(cameras.size());
    for(auto camera : cameras){
        if(camera.second->model_name != "PINHOLE"){
            LOG(FATAL) << "Sorry, the code doesn't support any camera model other than PINHOLE!!!";
        }
        int camera_id = camera.second->camera_id;
        double fx = camera.second->parameters[0];
        double fy = camera.second->parameters[1];
        double cx = camera.second->parameters[2];
        double cy = camera.second->parameters[3];
        view_vectors_[camera_id].resize(camera.second->height);
        for(int i = 0; i < camera.second->height; i++){
            view_vectors_[camera_id][i].resize(camera.second->width);
            for(int j = 0; j < camera.second->width; j++){
                double pixel_u = j;
                double pixel_v = i;
                double u_n = (pixel_u - cx) / fx;
                double v_n = (pixel_v - cy) / fy;
                view_vectors_[camera_id][i][j] = Eigen::Vector3d(u_n, v_n, 1.0);
                // LOG(INFO) << view_vectors_[camera_id][i][j][0] << " " << view_vectors_[camera_id][i][j][1] << " " << view_vectors_[camera_id][i][j][2];
            }
        }
    }
}

MVSProcess::~MVSProcess(){
    frame_ptr_1.reset();
    frame_ptr_2.reset();
    frame_ptr_3.reset();
}

void MVSProcess::updateFrames(int id, ColmapImagePtrMap &map){
    LOG(INFO) << "Updating frames...";
    if(frame_ptr_2){
        frame_ptr_1 = frame_ptr_2;
        frame_ptr_2 = frame_ptr_3;
        frame_ptr_3.reset(new MVSFrame(id, image_root_path_, map));
    }
    else{
        frame_ptr_2 = frame_ptr_3;
        frame_ptr_3.reset(new MVSFrame(id, image_root_path_, map));
    }
}

Eigen::Vector3d MVSProcess::getViewVector(int camera_id, int row, int col){
    if(view_vectors_.find(camera_id) != view_vectors_.end()){
        LOG(FATAL) << "Didn't find camera ID " << camera_id;
    }

    return view_vectors_[camera_id][row][col];
}

void MVSProcess::initializePlanes(){
    if(!frame_ptr_3){
        LOG(INFO) << "Frame 3 is empty. Pass...";
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_for_z(set_parameters_.min_z_, set_parameters_.max_z_);
    std::uniform_real_distribution<double> dis_for_vector(-1.0, 1.0);
    for(int i = 0; i < frame_ptr_3->pixels.rows; i++){
        for(int j = 0; j < frame_ptr_3->pixels.cols; j++){

            double q1, q2, S = 2.0;
            while(S >= 1.0){
                q1 = dis_for_vector(gen);
                q2 = dis_for_vector(gen);
                S = q1 * q1 + q2 * q2;
            }
            double sq = sqrtf(1.0 - S);
            frame_ptr_3->planes[i][j].x = 2.0 + q1 + sq;
            frame_ptr_3->planes[i][j].y = 2.0 + q2 + sq;
            frame_ptr_3->planes[i][j].z = 1.0 - 2.0 * S;
            Eigen::Vector3d normal_vector(2.0 + q1 + sq, 2.0 + q2 + sq, 1.0 - 2.0 * S);

            double random_z = dis_for_z(gen);
            Eigen::Vector3d cur_pixel_view_vector = getViewVector(frame_ptr_3->camera_id, i, j);
            frame_ptr_3->planes[i][j].w = cur_pixel_view_vector.dot(normal_vector) * random_z; 

        }
    }
}