#include "MVSProcess.hpp"

MVSFrame::MVSFrame(int id, std::string image_root_path, ColmapImagePtrMap &map, float zoom) : image_id(id) {
    ColmapImagePtr image_ptr = map[image_id];
    camera_id = image_ptr->camera_id;
    // LOG(INFO) << "camera_id: " << camera_id;
    // LOG(INFO) << "image_T_global: " << image_T_global.matrix();
    // image_T_global = image_ptr->image_T_global;
    image_T_global.linear() = image_ptr->image_T_global.linear();
    image_T_global.translation() = image_ptr->image_T_global.translation();
    // global_T_image = image_ptr->global_T_image;
    global_T_image.linear() = image_ptr->global_T_image.linear();
    global_T_image.translation() = image_ptr->global_T_image.translation();

    std::string image_file_path = image_ptr->file_path;
    LOG(INFO) << "Read image " << image_file_path;
    cv::Mat image_tmp;
    image_tmp = cv::imread(image_root_path + "/" + image_file_path, cv::IMREAD_COLOR);
    if (image_tmp.empty()) {
        LOG(FATAL) << "Could not read the image: " << image_root_path;
    }
    cv::resize(image_tmp, pixels_color, cv::Size(), zoom, zoom);
    cv::cvtColor(pixels_color, pixels, cv::COLOR_BGR2GRAY);

    // initialize planes whose size is the same as pixels' size
    planes.resize(pixels.rows, std::vector<MVSPlane>(pixels.cols, MVSPlane()));
}

MVSProcess::MVSProcess(std::string image_root_path, SetParameters set_parameters, ColmapCameraPtrMap &cameras,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
    : image_root_path_(image_root_path), set_parameters_(set_parameters), cameras_(cameras), cloud(_cloud) {
    // view_vectors_.resize(cameras.size());
    // cameras_ = cameras;
    LOG(INFO) << "Initialize view vectors. It may take for a while...";
    clock_t start_time = std::clock();
    for (auto &camera : cameras) {
        if (camera.second->model_name != "PINHOLE") {
            LOG(FATAL) << "Sorry, the code doesn't support any camera model other than PINHOLE!!!";
        }
        int camera_id = camera.second->camera_id;
        for (auto &item : camera.second->parameters) {
            item *= set_parameters_.zoom;
        }
        double fx = camera.second->parameters[0];
        double fy = camera.second->parameters[1];
        double cx = camera.second->parameters[2];
        double cy = camera.second->parameters[3];
        view_vectors_[camera_id].resize(camera.second->height);
        for (int i = 0; i < camera.second->height; i++) {
            view_vectors_[camera_id][i].resize(camera.second->width);
            for (int j = 0; j < camera.second->width; j++) {
                double pixel_u = j;
                double pixel_v = i;
                double u_n = (pixel_u - cx) / fx;
                double v_n = (pixel_v - cy) / fy;
                view_vectors_[camera_id][i][j] = Eigen::Vector3d(u_n, v_n, 1.0);
                // LOG(INFO) << view_vectors_[camera_id][i][j][0] << " " << view_vectors_[camera_id][i][j][1] << " " <<
                // view_vectors_[camera_id][i][j][2];
            }
        }
    }
    clock_t end_time = std::clock();
    LOG(INFO) << "View vectors Initialization is down. The elapsed time is "
              << (end_time - start_time) * 1.0 / CLOCKS_PER_SEC << " second(s)";
}

MVSProcess::~MVSProcess() {
    frame_ptr_1.reset();
    frame_ptr_2.reset();
    frame_ptr_3.reset();
}

void MVSProcess::updateFrames(int id, ColmapImagePtrMap &map) {
    LOG(INFO) << "Updating frames...";
    if (frame_ptr_2) {
        frame_ptr_1 = frame_ptr_2;
        frame_ptr_2 = frame_ptr_3;
        frame_ptr_3.reset(new MVSFrame(id, image_root_path_, map, set_parameters_.zoom));
    } else {
        frame_ptr_2 = frame_ptr_3;
        frame_ptr_3.reset(new MVSFrame(id, image_root_path_, map, set_parameters_.zoom));
    }
    std::string frame_3_id_str = frame_ptr_3 ? std::to_string(frame_ptr_3->image_id) + " " : "";
    std::string frame_2_id_str = frame_ptr_2 ? std::to_string(frame_ptr_2->image_id) + " " : "";
    std::string frame_1_id_str = frame_ptr_1 ? std::to_string(frame_ptr_1->image_id) + " " : "";
    LOG(INFO) << "Currently, Frame " << frame_3_id_str << frame_2_id_str << frame_1_id_str
              << " are(is) in the sequence";
}

Eigen::Vector3d MVSProcess::getViewVector(int camera_id, int row, int col) {
    if (view_vectors_.find(camera_id) == view_vectors_.end()) {
        LOG(FATAL) << "Didn't find camera ID " << camera_id;
    }

    return view_vectors_[camera_id][row][col];
}

void MVSProcess::initializePlanes() {
    if (!frame_ptr_3) {
        LOG(INFO) << "frame_ptr_3 is empty. Pass...";
    } else {
        LOG(INFO) << "Initialize depth planes for frame_ptr_3 whose actual image_id is " << frame_ptr_3->image_id;
    }

    LOG(INFO) << "Plane initialization may take for a while...";
    clock_t start_time = std::clock();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_for_z(set_parameters_.min_z_, set_parameters_.max_z_);
    std::uniform_real_distribution<double> dis_for_vector(-1.0, 1.0);
    for (int i = 0; i < frame_ptr_3->pixels.rows; i++) {
        for (int j = 0; j < frame_ptr_3->pixels.cols; j++) {

            double q1, q2, S = 2.0;
            while (S >= 1.0) {
                q1 = dis_for_vector(gen);
                q2 = dis_for_vector(gen);
                S = q1 * q1 + q2 * q2;
            }
            double sq = sqrtf(1.0 - S);
            Eigen::Vector3d cur_pixel_view_vector = getViewVector(frame_ptr_3->camera_id, i, j);
            Eigen::Vector3d normal_vector(2.0 + q1 + sq, 2.0 + q2 + sq, 1.0 - 2.0 * S);
            if (cur_pixel_view_vector.dot(normal_vector) > 0) {
                normal_vector = -normal_vector;
            }
            frame_ptr_3->planes[i][j].x = normal_vector[0];
            frame_ptr_3->planes[i][j].y = normal_vector[1];
            frame_ptr_3->planes[i][j].z = normal_vector[2];
            // Eigen::Vector3d normal_vector(2.0 + q1 + sq, 2.0 + q2 + sq, 1.0 - 2.0 * S);

            double random_z = dis_for_z(gen);
            // Eigen::Vector3d cur_pixel_view_vector = getViewVector(frame_ptr_3->camera_id, i, j);
            frame_ptr_3->planes[i][j].w = -cur_pixel_view_vector.dot(normal_vector) * random_z;
            // if (cur_pixel_view_vector.dot(normal_vector) * frame_ptr_3->planes[i][j].w > 0) {
            //     frame_ptr_3->planes[i][j].w = -frame_ptr_3->planes[i][j].w;
            // }
        }
    }
    clock_t end_time = std::clock();
    LOG(INFO) << "Plane Initialization is down. The elapsed time is " << (end_time - start_time) * 1.0 / CLOCKS_PER_SEC
              << " second(s)";
}

bool MVSProcess::projectToCamera(Eigen::Vector3f global_point, std::shared_ptr<MVSFrame> frame_ptr, size_t &pixel_x,
                                 size_t &pixel_y) {
    Eigen::Vector3f point_in_frame = frame_ptr->image_T_global * global_point;
    auto camera = cameras_[frame_ptr->camera_id];
    double fx = camera->parameters[0];
    double fy = camera->parameters[1];
    double cx = camera->parameters[2];
    double cy = camera->parameters[3];
    float px = fx * point_in_frame[0] / point_in_frame[2] + cx;
    float py = fy * point_in_frame[1] / point_in_frame[2] + cy;

    if (px <= 0 || px >= static_cast<float>(frame_ptr->cols() - 1) || py <= 0 ||
        py >= static_cast<float>(frame_ptr->rows() - 1)) {
        return false;
    } else {
        pixel_x = static_cast<size_t>(px);
        pixel_y = static_cast<size_t>(py);
        return true;
    }
}

float MVSProcess::calculateProjectionCost(Eigen::Vector3d view_vector, MVSPlane plane, float val_original,
                                          float grad_horizon_original, float grad_vertical_original) {
    Eigen::Vector3d local_point_3D = view_vector * (-plane.w / view_vector.dot(plane.vector()));
    Eigen::Vector3f global_point_3D = frame_ptr_2->global_T_image * local_point_3D.cast<float>();
    size_t pixel_x_in_frame_3, pixel_y_in_frame_3;
    if (projectToCamera(global_point_3D, frame_ptr_3, pixel_x_in_frame_3, pixel_y_in_frame_3)) {
        float val_projection = frame_ptr_3->pixels.at<uchar>(pixel_y_in_frame_3, pixel_x_in_frame_3);
        float grad_horizon_projection = frame_ptr_3->pixels.at<uchar>(pixel_y_in_frame_3, pixel_x_in_frame_3 + 1) -
                                        frame_ptr_3->pixels.at<uchar>(pixel_y_in_frame_3, pixel_x_in_frame_3 - 1);
        float grad_vertical_projection = frame_ptr_3->pixels.at<uchar>(pixel_y_in_frame_3 + 1, pixel_x_in_frame_3) -
                                         frame_ptr_3->pixels.at<uchar>(pixel_y_in_frame_3 - 1, pixel_x_in_frame_3);

        float valdiff = fabs(val_projection - val_original);
        float graddiff_horizon = fabs(grad_horizon_projection - grad_horizon_original);
        float graddiff_vertical = fabs(grad_vertical_projection - grad_vertical_original);
        float graddiff = 0.25f * (graddiff_horizon + graddiff_vertical);

        float gamma = 10.f;
        float val_cost = 1.f - expf(-valdiff / gamma);
        float grad_cost = 1.f - expf(-graddiff / gamma);
        float alpha = 0.3f;
        return alpha * val_cost + (1.f - alpha) * grad_cost;
    } else {
        return NON_OBSERVABLE_COST;
    }
}

float MVSProcess::calculatePlaneProjectionCost(int i, int j, MVSPlane plane) {
    float cost = 0;
    float num = 0;
    for (int row = i - set_parameters_.patch_length; row <= int(i + set_parameters_.patch_length);
         row += set_parameters_.patch_step) {
        for (int col = j - set_parameters_.patch_length; col <= int(j + set_parameters_.patch_length);
             col += set_parameters_.patch_step) {
            if (row <= 0 || row >= frame_ptr_2->rows() - 1 || col <= 0 || col >= frame_ptr_2->cols() - 1) {
                return NON_OBSERVABLE_COST;
            }
            float val = frame_ptr_2->pixels.at<uchar>(row, col);
            float grad_horizon =
                frame_ptr_2->pixels.at<uchar>(row, col + 1) - frame_ptr_2->pixels.at<uchar>(row, col - 1);
            float grad_vertical =
                frame_ptr_2->pixels.at<uchar>(row + 1, col) - frame_ptr_2->pixels.at<uchar>(row - 1, col);
            Eigen::Vector3d cur_view_vector = getViewVector(frame_ptr_2->camera_id, row, col);
            float cur_cost = calculateProjectionCost(cur_view_vector, plane, val, grad_horizon, grad_vertical);
            if (cur_cost >= NON_OBSERVABLE_COST * 0.5) {
                return NON_OBSERVABLE_COST;
            } else {
                cost += cur_cost;
                num += 1;
            }
        }
    }
    return cost / num;
}

void MVSProcess::saveAsDepthImage(std::shared_ptr<MVSFrame> frame_ptr, std::string output_filename) {
    size_t rows = frame_ptr->rows();
    size_t cols = frame_ptr->cols();

    cv::Mat depth_image(rows, cols, CV_8UC1);
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            MVSPlane &plane = frame_ptr->planes[i][j];
            Eigen::Vector3d view_vector = getViewVector(frame_ptr->camera_id, i, j);
            Eigen::Vector3d local_point_3D = view_vector * (-plane.w / view_vector.dot(plane.vector()));
            double depth = local_point_3D[2];
            // LOG(INFO) << depth;
            double scale = 20;
            uchar val;
            if (depth < 0 || depth >= scale) {
                val = 255;
            } else {
                val = static_cast<uchar>(depth / scale * 255);
            }
            depth_image.at<uchar>(i, j) = val;
        }
    }
    cv::imwrite(output_filename, depth_image);
}

void MVSProcess::refinePlane(size_t i, size_t j, float cost) {
    MVSPlane plane = frame_ptr_2->planes[i][j];
    Eigen::Vector3d cur_view_vector = getViewVector(frame_ptr_2->camera_id, i, j);
    // float depth = (cur_view_vector * (-plane.w / cur_view_vector.dot(plane.vector()))).norm();
    float depth = (cur_view_vector * (-plane.w / cur_view_vector.dot(plane.vector())))[2];
    Eigen::Vector3d plane_normalized_vector = plane.getNormalizedVector();
    float depth_range = 5;
    double vector_range = 1;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_for_vector(-vector_range, vector_range);
    // std::uniform_real_distribution<double> dis_for_z(set_parameters_.min_z_, set_parameters_.max_z_);
    // std::uniform_real_distribution<double> dis_for_vector(-1.0, 1.0);
    // float cost = FLT_MAX;
    while (depth_range > 0.05) {
        float random_depth_min =
            (depth - depth_range) > set_parameters_.min_z_ ? depth - depth_range : set_parameters_.min_z_;
        float random_depth_max =
            (depth + depth_range) < set_parameters_.max_z_ ? depth + depth_range : set_parameters_.max_z_;
        if (depth < set_parameters_.min_z_) {
            random_depth_min = set_parameters_.min_z_;
            random_depth_max = set_parameters_.min_z_ + depth_range;
        } else if (depth > set_parameters_.max_z_) {
            random_depth_min = set_parameters_.max_z_ - depth_range;
            random_depth_max = set_parameters_.max_z_;
        }
        std::uniform_real_distribution<float> dis_for_depth(random_depth_min, random_depth_max);
        float random_depth = dis_for_depth(gen);
        // std::uniform_real_distribution<double> dis_for_vector(-vector_range, vector_range);
        // float random_depth = std::max<float>(dis_for_depth(gen), set_parameters_.min_z_);
        // random_depth = std::min<float>(random_depth, set_parameters_.max_z_);
        Eigen::Vector3d random_vector =
            plane_normalized_vector + Eigen::Vector3d(dis_for_vector(gen), dis_for_vector(gen), dis_for_vector(gen));
        random_vector.normalize();
        if (random_vector.dot(cur_view_vector) > 0) {
            random_vector = -random_vector;
        }
        MVSPlane random_plane;
        random_plane.x = random_vector[0];
        random_plane.y = random_vector[1];
        random_plane.z = random_vector[2];
        random_plane.w = -(depth * cur_view_vector).dot(random_vector);
        float cost_candidate = calculatePlaneProjectionCost(i, j, random_plane);
        if (cost_candidate < cost) {
            cost = cost_candidate;
            frame_ptr_2->planes[i][j] = random_plane;
            depth = random_depth;
            plane_normalized_vector = random_plane.getNormalizedVector();
        }
        depth_range /= 2.0;
        vector_range /= 2.0;
    }
}

void MVSProcess::propagatePlanesFromPreviousFrame() {
    LOG(INFO) << "propagating planes from previous frame";
    for (int i = 0; i < frame_ptr_1->rows(); i++) {
        for (int j = 0; j < frame_ptr_1->cols(); j++) {
            MVSPlane plane = frame_ptr_1->planes[i][j];
            Eigen::Vector3d view_vector = getViewVector(frame_ptr_1->camera_id, i, j);
            Eigen::Vector3d local_point_3D = view_vector * (-plane.w / view_vector.dot(plane.vector()));
            Eigen::Vector3f global_point_3D = frame_ptr_1->global_T_image * local_point_3D.cast<float>();
            size_t pixel_x_in_frame2, pixel_y_in_frame2;
            if (projectToCamera(global_point_3D, frame_ptr_2, pixel_x_in_frame2, pixel_y_in_frame2)) {
                float cost = calculatePlaneProjectionCost(pixel_y_in_frame2, pixel_x_in_frame2,
                                                          frame_ptr_2->planes[pixel_y_in_frame2][pixel_x_in_frame2]);
                float cost_candidate = calculatePlaneProjectionCost(pixel_y_in_frame2, pixel_x_in_frame2, plane);
                if (cost_candidate < cost) {
                    frame_ptr_2->planes[pixel_y_in_frame2][pixel_x_in_frame2] = plane;
                }
            }
        }
    }
}

void MVSProcess::planePropagation() {
    if (!frame_ptr_2) {
        LOG(INFO) << "frame_ptr_2 is empty, pass!";
    } else {
        LOG(INFO) << "start plane propagation for frame_ptr_2 whose actual image_id is " << frame_ptr_2->image_id;
        clock_t start_time = std::clock();
        // LOG(INFO) << "image size for frame_ptr_2: (" << frame_ptr_2->rows() << ", " << frame_ptr_2->cols() << ")";
        saveAsDepthImage(frame_ptr_2,
                         "/home/ziyunge/" + std::to_string(frame_ptr_2->image_id) + "_before_propagation.jpg");
        if (frame_ptr_1) {
            propagatePlanesFromPreviousFrame();
        }
        for (int loop_time = 0; loop_time < 3; loop_time++) {
            clock_t time_in_loop = std::clock();
            for (size_t i = 1; i < frame_ptr_2->rows() - 1; i++) {
                for (size_t j = 1; j < frame_ptr_2->cols() - 1; j++) {
                    // LOG(INFO) << "Processing (" << i << ", " << j << ")";
                    // float val = frame_ptr_2->pixels.at<uchar>(i, j);
                    // LOG(INFO) << val;
                    // float grad_horizon =
                    //     frame_ptr_2->pixels.at<uchar>(i, j + 1) - frame_ptr_2->pixels.at<uchar>(i, j - 1);
                    // float grad_vertical =
                    //     frame_ptr_2->pixels.at<uchar>(i + 1, j) - frame_ptr_2->pixels.at<uchar>(i - 1, j);

                    // Eigen::Vector3d cur_view_vector = getViewVector(frame_ptr_2->camera_id, i, j);
                    MVSPlane plane = frame_ptr_2->planes[i][j];
                    // float cost = calculateProjectionCost(cur_view_vector, plane, val, grad_horizon, grad_vertical);
                    float cost = calculatePlaneProjectionCost(i, j, plane);
                    float cost_candidate;

                    // for (int offseti = -set_parameters_.patch_length; offseti <= 0;
                    //      offseti += set_parameters_.patch_step * 2) {
                    //     for (int offsetj = -set_parameters_.patch_length; offsetj <= set_parameters_.patch_length;
                    //          offsetj += set_parameters_.patch_step * 2) {
                    //         if (offseti <= 1 && offseti >= -1 && offsetj <= 1 && offsetj >= -1) {
                    //             continue;
                    //         }
                    //         int selected_i = i + offseti;
                    //         int selected_j = j + offsetj;
                    //         if (selected_i < 0 || selected_i >= frame_ptr_2->rows() || selected_j < 0 ||
                    //             selected_j >= frame_ptr_2->cols()) {
                    //             continue;
                    //         }
                    //         plane = frame_ptr_2->planes[selected_i][selected_j];
                    //         cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    //         if (cost_candidate < cost) {
                    //             cost = cost_candidate;
                    //             frame_ptr_2->planes[i][j] = plane;
                    //         }
                    //     }
                    // }

                    // srand((unsigned)time(NULL));
                    // for (int random_try = 0; random_try < 5; random_try++) {
                    //     int offseti = 0;
                    //     int offsetj = 0;
                    //     while (offseti >= -1 && offseti <= 1 && offsetj >= -1 && offsetj <= 1) {
                    //         offseti = rand() % (set_parameters_.patch_length + 1) - set_parameters_.patch_length;
                    //         offsetj = rand() % (2 * set_parameters_.patch_length + 1) - set_parameters_.patch_length;
                    //     }
                    //     int selected_i = i + offseti;
                    //     int selected_j = j + offsetj;
                    //     if (selected_i < 0 || selected_i >= frame_ptr_2->rows() || selected_j < 0 ||
                    //         selected_j >= frame_ptr_2->cols()) {
                    //         continue;
                    //     }
                    //     plane = frame_ptr_2->planes[selected_i][selected_j];
                    //     cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    //     if (cost_candidate < cost) {
                    //         cost = cost_candidate;
                    //         frame_ptr_2->planes[i][j] = plane;
                    //     }
                    // }

                    plane = frame_ptr_2->planes[i][j - 1]; // left plane
                                                           // float cost_candidate =
                    //     calculateProjectionCost(cur_view_vector, plane, val, grad_horizon, grad_vertical);
                    cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    if (cost_candidate < cost) {
                        cost = cost_candidate;
                        frame_ptr_2->planes[i][j] = plane;
                    }

                    plane = frame_ptr_2->planes[i - 1][j]; // up plane
                    // cost_candidate = calculateProjectionCost(cur_view_vector, plane, val, grad_horizon,
                    // grad_vertical);
                    cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    if (cost_candidate < cost) {
                        cost = cost_candidate;
                        frame_ptr_2->planes[i][j] = plane;
                    }

                    refinePlane(i, j, cost);
                }
            }
            saveAsDepthImage(frame_ptr_2, "/home/ziyunge/" + std::to_string(frame_ptr_2->image_id) +
                                              "_after_forward_propagation_" + std::to_string(loop_time) + ".jpg");

            for (size_t i = frame_ptr_2->rows() - 2; i >= 1; i--) {
                for (size_t j = frame_ptr_2->cols() - 2; j >= 1; j--) {
                    // LOG(INFO) << "Processing (" << i << ", " << j << ")";
                    // float val = frame_ptr_2->pixels.at<uchar>(i, j);
                    // LOG(INFO) << val;
                    // float grad_horizon =
                    //     frame_ptr_2->pixels.at<uchar>(i, j + 1) - frame_ptr_2->pixels.at<uchar>(i, j - 1);
                    // float grad_vertical =
                    //     frame_ptr_2->pixels.at<uchar>(i + 1, j) - frame_ptr_2->pixels.at<uchar>(i - 1, j);

                    // Eigen::Vector3d cur_view_vector = getViewVector(frame_ptr_2->camera_id, i, j);
                    MVSPlane plane = frame_ptr_2->planes[i][j];
                    // float cost = calculateProjectionCost(cur_view_vector, plane, val, grad_horizon, grad_vertical);
                    float cost = calculatePlaneProjectionCost(i, j, plane);
                    float cost_candidate;

                    // for (int offseti = 0; offseti <= set_parameters_.patch_length;
                    //      offseti += set_parameters_.patch_step * 2) {
                    //     for (int offsetj = -set_parameters_.patch_length; offsetj <= set_parameters_.patch_length;
                    //          offsetj += set_parameters_.patch_step * 2) {
                    //         if (offseti <= 1 && offseti >= -1 && offsetj <= 1 && offsetj >= -1) {
                    //             continue;
                    //         }
                    //         int selected_i = i + offseti;
                    //         int selected_j = j + offsetj;
                    //         if (selected_i < 0 || selected_i >= frame_ptr_2->rows() || selected_j < 0 ||
                    //             selected_j >= frame_ptr_2->cols()) {
                    //             continue;
                    //         }
                    //         plane = frame_ptr_2->planes[selected_i][selected_j];
                    //         cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    //         if (cost_candidate < cost) {
                    //             cost = cost_candidate;
                    //             frame_ptr_2->planes[i][j] = plane;
                    //         }
                    //     }
                    // }

                    // srand((unsigned)time(NULL));
                    // for (int random_try = 0; random_try < 5; random_try++) {
                    //     int offseti = 0;
                    //     int offsetj = 0;
                    //     while (offseti >= -1 && offseti <= 1 && offsetj >= -1 && offsetj <= 1) {
                    //         offseti = rand() % (set_parameters_.patch_length + 1);
                    //         offsetj = rand() % (2 * set_parameters_.patch_length + 1) - set_parameters_.patch_length;
                    //     }
                    //     int selected_i = i + offseti;
                    //     int selected_j = j + offsetj;
                    //     if (selected_i < 0 || selected_i >= frame_ptr_2->rows() || selected_j < 0 ||
                    //         selected_j >= frame_ptr_2->cols()) {
                    //         continue;
                    //     }
                    //     plane = frame_ptr_2->planes[selected_i][selected_j];
                    //     cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    //     if (cost_candidate < cost) {
                    //         cost = cost_candidate;
                    //         frame_ptr_2->planes[i][j] = plane;
                    //     }
                    // }

                    plane = frame_ptr_2->planes[i][j + 1]; // right plane
                                                           // float cost_candidate =
                    //     calculateProjectionCost(cur_view_vector, plane, val, grad_horizon, grad_vertical);
                    cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    if (cost_candidate < cost) {
                        cost = cost_candidate;
                        frame_ptr_2->planes[i][j] = plane;
                    }

                    plane = frame_ptr_2->planes[i + 1][j]; // down plane
                    // cost_candidate = calculateProjectionCost(cur_view_vector, plane, val, grad_horizon,
                    // grad_vertical);
                    cost_candidate = calculatePlaneProjectionCost(i, j, plane);
                    if (cost_candidate < cost) {
                        cost = cost_candidate;
                        frame_ptr_2->planes[i][j] = plane;
                    }

                    refinePlane(i, j, cost);
                }
            }
            saveAsDepthImage(frame_ptr_2, "/home/ziyunge/" + std::to_string(frame_ptr_2->image_id) +
                                              "_after_backward_propagation_" + std::to_string(loop_time) + ".jpg");
            // exit(0);
            LOG(INFO) << "Loop " + std::to_string(loop_time) + " ended. The elapsed time is "
                      << (std::clock() - time_in_loop) * 1.0 / CLOCKS_PER_SEC / 60.0 << " min(s)";
        }
        LOG(INFO) << "Finished plane propagation. The elapsed time is "
                  << (std::clock() - start_time) * 1.0 / CLOCKS_PER_SEC / 60.0 << " min(s)";
    }
}

void MVSProcess::checkConsistency() {
    if (!frame_ptr_1) {
        LOG(INFO) << "frame_ptr_1 is empty, pass!";
    } else {
        LOG(INFO) << "start check consistency for frame_ptr_2 whose actual image_id is " << frame_ptr_2->image_id
                  << ". The reference image_id is " << frame_ptr_1->image_id;
        clock_t start_time = std::clock();
        for (size_t i = 0; i < frame_ptr_2->rows(); i++) {
            for (size_t j = 0; j < frame_ptr_2->cols(); j++) {
                Eigen::Vector3d cur_view_vector = getViewVector(frame_ptr_2->camera_id, i, j);
                MVSPlane cur_plane = frame_ptr_2->planes[i][j];
                Eigen::Vector3d cur_local_point_3D =
                    cur_view_vector * (-cur_plane.w / cur_view_vector.dot(cur_plane.vector()));
                Eigen::Vector3f cur_global_point_3D = frame_ptr_2->global_T_image * cur_local_point_3D.cast<float>();

                size_t pixel_x_in_frame_1, pixel_y_in_frame_1;
                if (projectToCamera(cur_global_point_3D, frame_ptr_1, pixel_x_in_frame_1, pixel_y_in_frame_1)) {
                    Eigen::Vector3d projection_view_vector =
                        getViewVector(frame_ptr_1->camera_id, pixel_y_in_frame_1, pixel_x_in_frame_1);
                    MVSPlane projection_plane = frame_ptr_1->planes[pixel_y_in_frame_1][pixel_x_in_frame_1];
                    Eigen::Vector3d projection_local_point_3D =
                        projection_view_vector *
                        (-projection_plane.w / projection_view_vector.dot(projection_plane.vector()));
                    Eigen::Vector3f projection_global_point_3D =
                        frame_ptr_1->global_T_image * projection_local_point_3D.cast<float>();

                    if ((projection_global_point_3D - cur_global_point_3D).norm() <= 0.05 &&
                        std::abs(cur_plane.getNormalizedVector().dot(projection_plane.getNormalizedVector())) >
                            0.984808) {
                        // collect good points
                        pcl::PointXYZRGB p;
                        p.x = cur_global_point_3D[0];
                        p.y = cur_global_point_3D[1];
                        p.z = cur_global_point_3D[2];
                        p.b = frame_ptr_2->pixels_color.at<cv::Vec3b>(i, j)[0];
                        p.g = frame_ptr_2->pixels_color.at<cv::Vec3b>(i, j)[1];
                        p.r = frame_ptr_2->pixels_color.at<cv::Vec3b>(i, j)[2];
                        cloud->push_back(p);
                    }
                }
            }
        }
        LOG(INFO) << "Finished check consistency. The elapsed time is "
                  << (std::clock() - start_time) * 1.0 / CLOCKS_PER_SEC << " second(s)";
    }
}