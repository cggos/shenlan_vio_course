#include <iostream>
#include <random>
#include "backend/vertex_inverse_depth.h"
#include "backend/vertex_pose.h"
#include "backend/edge_reprojection.h"
#include "backend/problem.h"

using namespace myslam::backend;
using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex(): Vertex(3) {}
    virtual std::string TypeInfo() const { return "abc"; }
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x, double y ): Edge(1,1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }
    // 计算曲线模型误差
    virtual void ComputeResidual() override
    {
        Vec3 abc = verticies_[0]->Parameters();
        residual_(0) = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) ) - y_;
    }
    virtual void ComputeJacobians() override
    {
        Vec3 abc = verticies_[0]->Parameters();
        double exp_y = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) );
        Eigen::Matrix<double, 1, 3> jaco_abc;
        jaco_abc << x_ * x_ * exp_y, x_ * exp_y , 1 * exp_y;
        jacobians_[0] = jaco_abc;
    }
    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "CurveFittingEdge"; }
public:
    double x_,y_;  // x 值， y 值为 _measurement
};

int main()
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N = 100;                          // 数据点
    double w_sigma= 1.;                 // 噪声Sigma值

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    // 构建 problem
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    shared_ptr< CurveFittingVertex > vertex(new CurveFittingVertex());
    vertex->SetParameters(Eigen::Vector3d (0.,0.,0.));

    problem.AddVertex(vertex);
    for (int i = 0; i < N; ++i) {
        double x = i/100.;
        double n = noise(generator);
        double y = std::exp( a*x*x + b*x + c ) + n;
//        double y = std::exp( a*x*x + b*x + c );
        shared_ptr< CurveFittingEdge > edge(new CurveFittingEdge(x,y));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);

        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;
    problem.Solve(30);
    std::cout << vertex->Parameters() << std::endl;
    return 0;
}


