
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/cg/projects/vio_class/ch2_code/vio_data_simulation-master/bin/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

void PubImuData()
{
	string sImu_data_file = sData_path + "imu_pose_noise.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec
		          >> q.w() >> q.x() >> q.y() >> q.z() >> t(0) >> t(1) >> t(2)
		          >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc);
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
	string sImage_file = sData_path + "cam_pose.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampSec;
	std::vector<double> vdStampSec;
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImgData(sImage_line);
		ssImgData >> dStampSec;
		vdStampSec.push_back(dStampSec);
	}
	fsImage.close();

	for(int i=0; i<vdStampSec.size(); i++) {
		dStampSec = vdStampSec[i];

		std::stringstream filename1;
        filename1 << sData_path << "keyframe/all_points_" << i << ".txt";

		Mat img;
		pSystem->PubImageData(dStampSec, filename1.str());

		usleep(50000*nDelayTimes);
	}
}

int main(int argc, char **argv)
{
	if(argc != 2)
	{
		cerr << "./run_sim PATH_TO_SIMDATA_FOLDER \n" 
			<< "For example: ./run_sim /home/cg/projects/vio_class/ch2_code/vio_data_simulation-master/bin/"<< endl;
		return -1;
	}
	sData_path = argv[1];

	pSystem.reset(new System(sConfig_path));
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);
	
	std::thread thd_Draw(&System::Draw, pSystem);
	
	thd_PubImuData.join();
	thd_PubImageData.join();

	thd_BackEnd.join();
	thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
