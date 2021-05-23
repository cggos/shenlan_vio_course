//
// Created by gaoxiang19 on 19-1-7.
//

#ifndef SLAM_COURSE_CAMERA_H
#define SLAM_COURSE_CAMERA_H

#include "backend/eigen_types.h"
#include "sophus/se3.hpp"

namespace myslam {
namespace frontend {

struct PinholeCamera {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PinholeCamera(
        const float &_fx, const float &_fy,
        const float &_cx, const float &_cy,
        const Sophus::SE3 &extrinsics,
        const float _bf = 0)
        : fx(_fx), fy(_fy), cx(_cx), cy(_cy), bf(_bf) {
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        fxinv = 1 / fx;
        fyinv = 1 / fy;
        Kinv = K.inverse();
        f = (fx + fy) * 0.5;
        b = bf / f;
        this->extrinsic = extrinsics;
    }

    float fx = 0;
    float fy = 0;
    float fxinv = 0;
    float fyinv = 0;
    float cx = 0;
    float cy = 0;
    float b = 0;    // baseline in stereo
    float f = 0;    // focal length
    float bf = 0;   // baseline*focal

    Eigen::Matrix3f K = Eigen::Matrix3f::Identity();     // intrinsics
    Eigen::Matrix3f Kinv = Eigen::Matrix3f::Identity();  // inverse K

    Sophus::SE3d extrinsic; // extrinsics
};

}
}

#endif //SLAM_COURSE_CAMERA_H
