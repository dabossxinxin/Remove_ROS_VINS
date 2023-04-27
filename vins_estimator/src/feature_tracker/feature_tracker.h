#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "../parameters.h"
#include "../utility/tic_toc.h"

using namespace camodocal;

bool inBorder(const cv::Point2f &pt);

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
void reduceVector(std::vector<int> &v, std::vector<uchar> status);

class FeatureTracker
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeatureTracker();

    void readImage(const cv::Mat &_img);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const std::string &calib_file);

    void showUndistortion(const std::string &name);

    void rejectWithF();

	std::vector<cv::Point2f> undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
	std::vector<cv::Point2f> n_pts;
	std::vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
	std::vector<int> ids;
	std::vector<int> track_cnt;
    camodocal::CameraPtr m_camera;

    static int n_id;
};
