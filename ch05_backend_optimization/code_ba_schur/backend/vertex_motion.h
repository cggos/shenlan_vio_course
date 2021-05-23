#ifndef MYSLAM_BACKEND_MOTIONVERTEX_H
#define MYSLAM_BACKEND_MOTIONVERTEX_H

#include <memory>
#include "backend/vertex.h"

namespace myslam {
namespace backend {

/**
 * Motion vertex
 * parameters: v, ba, bg 9 DoF
 * 
 */
class VertexMotion : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexMotion() : Vertex(9) {}

    std::string TypeInfo() const {
        return "VertexMotion";
    }

};

}
}

#endif
