#pragma once 

#include <vector>
#include "../parameters.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace Eigen;

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	InitialEXRotation();
    bool CalibrationExRotation(std::vector<std::pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
private:
	Matrix3d solveRelativeR(const std::vector<std::pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const std::vector<cv::Point2f> &l,
                             const std::vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
    int frame_count;

	std::vector< Matrix3d > Rc;
	std::vector< Matrix3d > Rimu;
	std::vector< Matrix3d > Rc_g;
    Matrix3d ric;
};


