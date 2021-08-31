// Copyright 2017 Thomas Schöps
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "images.hpp"

#include <fstream>
#include <iostream>

bool ReadColmapImages(const std::string &images_txt_path, bool read_observations, ColmapImagePtrMap &images) {
    std::ifstream images_file_stream(images_txt_path, std::ios::in);
    if (!images_file_stream) {
        return false;
    }
    while (!images_file_stream.eof() && !images_file_stream.bad()) {
        std::string line;
        std::getline(images_file_stream, line);
        if (line.size() == 0 || line[0] == '#') {
            continue;
        }

        // Read image info line.
        ColmapImagePtr new_image(new ColmapImage());
        Eigen::Quaternionf image_R_global;
        std::istringstream image_stream(line);
        image_stream >> new_image->image_id >> image_R_global.w() >> image_R_global.x() >> image_R_global.y() >>
            image_R_global.z() >> new_image->image_T_global.translation()[0] >>
            new_image->image_T_global.translation()[1] >> new_image->image_T_global.translation()[2] >>
            new_image->camera_id >> new_image->file_path;
        new_image->image_T_global.linear() = image_R_global.toRotationMatrix();
        // std::cout << new_image->image_T_global.translation().transpose() << std::endl;
        new_image->global_T_image = new_image->image_T_global.inverse();
        // std::cout << new_image->global_T_image.translation().transpose() << std::endl;

        // Read feature observations line.
        std::getline(images_file_stream, line);
        if (read_observations) {
            std::istringstream observations_stream(line);
            while (!observations_stream.eof() && !observations_stream.bad()) {
                new_image->observations.emplace_back();
                ColmapFeatureObservation *new_observation = &new_image->observations.back();
                observations_stream >> new_observation->xy.x() >> new_observation->xy.y() >>
                    new_observation->point3d_id;
            }
        }

        // std::cout << "new_image count " << new_image.use_count() << std::endl;
        images[new_image->image_id] = new_image;
        // std::cout << "new_image count " << new_image.use_count() << std::endl;
        // images.insert(std::make_pair(new_image->image_id, new_image));
        // std::cout << images[new_image->image_id]->global_T_image.translation().transpose() << std::endl << std::endl;
    }

    // if (!images->empty()) {
    //     SE3f globalnew_T_global = images->begin()->second->image_T_global;
    //     SE3f global_T_globalnew = images->begin()->second->global_T_image;
    //     for (auto &image : *images) {
    //         image.second->global_T_image = globalnew_T_global * image.second->global_T_image;
    //         image.second->image_T_global = image.second->image_T_global * global_T_globalnew;
    //     }
    // }
    return true;
}