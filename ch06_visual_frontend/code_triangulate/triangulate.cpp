//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};
int main()
{

    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
    }

    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */
    int num = end_frame_id − start_frame_id;
   Eigen::MatrixXd mA(2*num, 4);
   mA = Eigen::MatrixXd::Zero(2*num, 4);
   for( int i = 0 ; i < num; ++i ) {
   int idx = i + start_frame_id;
   double u = camera_pose[idx].uv[0];
   double v = camera_pose[idx].uv[1];
   Eigen::MatrixXd Tcw(3,4);
   Eigen::Matrix3d Rcw = camera_pose[idx].Rwc.transpose();
   Eigen::Vector3d tcw = −Rcw * camera_pose[idx].twc;
   Tcw.block<3,3>(0,0) = Rcw;
   Tcw.block<3 ,1>(0,3) = tcw ;
   mA.row( 2 * i + 0 ) = u * Tcw . row ( 2 ) − Tcw . row ( 0 ) ;
   mA.row( 2 * i + 1 ) = v * Tcw . row ( 2 ) − Tcw . row ( 1 ) ;
   }
   Eigen::JacobiSVD<Eigen::MatrixXd> svd (mA, Eigen::ComputeThinU | Eigen::ComputeThinV) ;
   Eigen::Matrix4d mV = svd.matrixV();
   Eigen::Vector4d v_last = mV.col( 3 ) ;
   if(v_last[3] < 1e−8)
   v_last [ 3 ] = 0.00001;
   P_est = v_last.hnormalized() ;
    /* your code end */

    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    
    return 0;
}
