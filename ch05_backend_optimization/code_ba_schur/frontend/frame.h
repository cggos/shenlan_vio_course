//
// Created by gaoxiang19 on 19-1-7.
//

#ifndef SLAM_COURSE_FRAME_H
#define SLAM_COURSE_FRAME_H

#include <opencv2/opencv.hpp>
#include <memory>
#include "camera.h"
#include "feature.h"
#include "backend/eigen_types.h"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

namespace myslam {
namespace frontend {

struct Frame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    double timestamp = 0.0;
    cv::Mat img_left, img_right;
    std::shared_ptr<PinholeCamera> camera = nullptr;

    std::vector<std::shared_ptr<Feature>> features;

    unsigned long frame_id = 0;
    unsigned long keyframe_id = 0;
    bool is_keyframe = false;
    std::weak_ptr<Frame> reference_keyframe;

    // poses and biases
    Sophus::SE3d Twb;
    Vec9 speed_and_bias = Vec9::Zero();
    Vec7 Twb_states;
    Vec9 speed_and_bias_states = Vec9::Zero();
};

}
}

#endif //SLAM_COURSE_FRAME_H
