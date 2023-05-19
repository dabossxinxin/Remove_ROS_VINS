#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "utility/print.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
//#include <std_msgs/Header.h>
//#include <std_msgs/Float32.h>
#include "../../include/Header.h"
#include "../../include/Float32.h"
#include <assert.h>
#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

struct RetriveData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	int		old_index;		// 回环帧ID
	int		cur_index;		// 当前帧ID
    double	header;			// 当前帧时间戳
	bool	relocalized;	// 当前信息是否已经用于重定位
	bool	relative_pose;	// 当前信息中是否有回环前后相同帧的位姿变换
	double	relative_yaw;	// 回环前后回环帧的YAW变换量
	double	loop_pose[7];	// 回环帧通过PnP计算得到的位姿
	Eigen::Vector3d				P_old;			// 回环帧位置
	Eigen::Matrix3d				R_old;			// 回环帧姿态
	Eigen::Vector3d				relative_t;		// 回环前后回环帧的位置变化量
	Eigen::Quaterniond			relative_q;		// 回环前后回环帧的姿态变化量
	std::vector<cv::Point2f>	measurements;	// 当前帧在回环中匹配到的特征点
	std::vector<int>			features_ids;	// 当前帧中匹配到的特征点ID
};

class Estimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Estimator();

	void setParameter();

	// interface
	void processIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
	void processImage(const std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>> &image, const std_msgs::Header &header);

	// internal
	void clearState();
	bool initialStructure();
	bool visualInitialAlign();
	bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);
	void slideWindow();
	void solveOdometry();
	void slideWindowNew();
	void slideWindowOld();
	void optimization();
	void vector2double();
	void double2vector();
	bool failureDetection();
	void localPointCloud();

	enum SolverFlag
	{
		INITIAL,
		NON_LINEAR
	};

	enum MarginalizationFlag
	{
		MARGIN_OLD = 0,
		MARGIN_SECOND_NEW = 1
	};

	SolverFlag solver_flag;
	MarginalizationFlag  marginalization_flag;
	Eigen::Vector3d g;
	Eigen::MatrixXd Ap[2], backup_A;
	Eigen::VectorXd bp[2], backup_b;

	Eigen::Matrix3d ric[NUM_OF_CAM];
	Eigen::Vector3d tic[NUM_OF_CAM];

	Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
	Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
	Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
	Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
	Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];

	Eigen::Matrix3d back_R0, last_R, last_R0;
	Eigen::Vector3d back_P0, last_P, last_P0;
	std_msgs::Header Headers[(WINDOW_SIZE + 1)];

	IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
	Eigen::Vector3d acc_0, gyr_0;

	std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
	std::vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
	std::vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

	int frame_count;
	int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

	FeatureManager f_manager;
	MotionEstimator m_estimator;
	InitialEXRotation initial_ex_rotation;

	bool first_imu;
	bool is_valid;
	bool is_key;
	bool failure_occur;

	std::vector<Eigen::Vector3d> point_cloud;
	std::vector<Eigen::Vector3d> margin_cloud;
	std::vector<Eigen::Vector3d> key_poses;
	double initial_timestamp;

	double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
	double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
	double para_Feature[NUM_OF_F][SIZE_FEATURE];
	double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
	double para_Retrive_Pose[SIZE_POSE];

	RetriveData retrive_pose_data, front_pose;
	std::vector<RetriveData> retrive_data_vector;
	int loop_window_index;
	bool relocalize;
	Eigen::Vector3d relocalize_t;
	Eigen::Matrix3d relocalize_r;

	MarginalizationInfo *last_marginalization_info;
	std::vector<double *> last_marginalization_parameter_blocks;

	std::map<double, ImageFrame> all_image_frame;
	IntegrationBase *tmp_pre_integration;
};