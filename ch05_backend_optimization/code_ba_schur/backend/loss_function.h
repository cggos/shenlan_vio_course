//
// Created by gaoxiang19 on 11/10/18.
//

#ifndef MYSLAM_LOSS_FUNCTION_H
#define MYSLAM_LOSS_FUNCTION_H

#include "backend/eigen_types.h"

namespace myslam {
namespace backend {

/**
 * loss function 即鲁棒核函数
 * loss套在误差之上
 * 假设某条边的残差为r，信息矩阵为I, 那么平方误差为r^T*I*r，令它的开方为e
 * 那么loss就是Compute(e)
 * 在求导时，也必须将loss function放到求导的第一项
 *
 * LossFunction是各核函数的基类，它可以派生出各种Loss
 */
class LossFunction {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual ~LossFunction() {}

    virtual double Compute(double error) const = 0;
};

/**
 * 平凡的Loss，不作任何处理
 * 使用nullptr作为loss function时效果相同
 *
 * TrivalLoss(e) = e^2
 */
class TrivalLoss : public LossFunction {
public:
    virtual double Compute(double error) const override { return error * error; }
};

/**
 * Huber loss
 *
 * Huber(e) = e^2                      if e <= delta
 * huber(e) = delta*(2*e - delta)      if e > delta
 */
class HuberLoss : public LossFunction {
public:
    explicit HuberLoss(double delta) : delta_(delta) {}

    virtual ~HuberLoss() {}

    virtual double Compute(double error) const override;

private:
    double delta_;

};

}
}

#endif //MYSLAM_LOSS_FUNCTION_H
