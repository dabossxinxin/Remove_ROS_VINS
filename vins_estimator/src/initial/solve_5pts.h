#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace Eigen;

//#include <ros/console.h>

class MotionEstimator
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool solveRelativeRT(const std::vector<std::pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

  private:
    double testTriangulation(const std::vector<cv::Point2f> &l,
                             const std::vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};