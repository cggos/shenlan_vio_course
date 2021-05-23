//
// Created by gaoxiang19 on 11/10/18.
//

#include "backend/loss_function.h"

namespace myslam {
namespace backend {

double HuberLoss::Compute(double error) const {
    if (error <= delta_) {
        return error * error;
    } else {
        return delta_ * (2 * error - delta_);
    }
}

}
}
