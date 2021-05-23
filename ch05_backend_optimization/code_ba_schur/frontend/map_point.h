//
// Created by gaoxiang19 on 19-1-7.
//

#ifndef SLAM_COURSE_MAP_POINT_H
#define SLAM_COURSE_MAP_POINT_H

namespace myslam {
namespace frontend {

struct Frame;
struct Feature;

struct MapPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    std::weak_ptr <Feature> ref_feature;
    float inv_depth = -1.0;
};

}
}

#endif //SLAM_COURSE_MAP_POINT_H
