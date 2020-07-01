/*
 * end_to_point_rad_cal.cpp
 *
 *  Created on: Oct 25, 2017
 *      Author: robotemperor
 */

#include "sdu_math/end_point_to_rad_cal.h"

EndEffectorTraj::EndEffectorTraj()
{
	cal_end_point_tra_px = new FifthOrderTrajectory;
	cal_end_point_tra_py = new FifthOrderTrajectory;
	cal_end_point_tra_pz = new FifthOrderTrajectory;
	cal_end_point_tra_alpha = new FifthOrderTrajectory;
	cal_end_point_tra_betta = new FifthOrderTrajectory;
	cal_end_point_tra_kamma = new FifthOrderTrajectory;
	cal_one_joint_traj_rad = new FifthOrderTrajectory;


	//current_pose_change.resize(6,3);
	current_pose_change.fill(0);

	current_one_joint_pose.resize(1,2);
	current_one_joint_pose.fill(0);

	//results.resize(6,1); // 6DOF LEG
	results.fill(0);

	is_moving_check = false;
}

EndEffectorTraj::~EndEffectorTraj()
{
  delete cal_end_point_tra_px;
  delete cal_end_point_tra_py;
  delete cal_end_point_tra_pz;
  delete cal_end_point_tra_alpha;
  delete cal_end_point_tra_betta;
  delete cal_end_point_tra_kamma;
  delete cal_one_joint_traj_rad;
}
void EndEffectorTraj::cal_end_point_to_rad(Eigen::Matrix<double, 6, 8> eP_) // end point 6 X 8 6 is xyz roll pitch yaw 8은 trajectory 입력
{
	if( cal_end_point_tra_px    -> detect_change_final_value(eP_(0,1), eP_(0,3), eP_(0,7))||
			cal_end_point_tra_py    -> detect_change_final_value(eP_(1,1), eP_(1,3), eP_(1,7))||
			cal_end_point_tra_pz    -> detect_change_final_value(eP_(2,1), eP_(2,3), eP_(2,7))||
			cal_end_point_tra_alpha -> detect_change_final_value(eP_(3,1), eP_(3,3), eP_(3,7))||
			cal_end_point_tra_betta -> detect_change_final_value(eP_(4,1), eP_(4,3), eP_(4,7))||
			cal_end_point_tra_kamma -> detect_change_final_value(eP_(5,1), eP_(5,3), eP_(5,7)) )
	{
		current_pose_change(0,0) = cal_end_point_tra_px    -> current_pose;
		current_pose_change(0,1) = cal_end_point_tra_px    -> current_velocity;
		current_pose_change(0,2) = cal_end_point_tra_px    -> current_acc;
		cal_end_point_tra_px    -> current_time = 0;

		current_pose_change(1,0) = cal_end_point_tra_py    -> current_pose;
		current_pose_change(1,1) = cal_end_point_tra_py    -> current_velocity;
		current_pose_change(1,2) = cal_end_point_tra_py    -> current_acc;
		cal_end_point_tra_py    -> current_time = 0;

		current_pose_change(2,0) = cal_end_point_tra_pz    -> current_pose;
		current_pose_change(2,1) = cal_end_point_tra_pz    -> current_velocity;
		current_pose_change(2,2) = cal_end_point_tra_pz    -> current_acc;
		cal_end_point_tra_pz    -> current_time = 0;

		current_pose_change(3,0) = cal_end_point_tra_alpha -> current_pose;
		current_pose_change(3,1) = cal_end_point_tra_alpha -> current_velocity;
		current_pose_change(3,2) = cal_end_point_tra_alpha    -> current_acc;
		cal_end_point_tra_alpha -> current_time = 0;

		current_pose_change(4,0) = cal_end_point_tra_betta -> current_pose;
		current_pose_change(4,1) = cal_end_point_tra_betta -> current_velocity;
		current_pose_change(4,2) = cal_end_point_tra_betta    -> current_acc;
		cal_end_point_tra_betta -> current_time = 0;

		current_pose_change(5,0) = cal_end_point_tra_kamma -> current_pose;
		current_pose_change(5,1) = cal_end_point_tra_kamma -> current_velocity;
		current_pose_change(5,2) = cal_end_point_tra_kamma    -> current_acc;
		cal_end_point_tra_kamma -> current_time = 0;

		//ROS_INFO("Initialize && Change End Point Value");
	}


	results(0,0) = cal_end_point_tra_px -> fifth_order_traj_gen(current_pose_change(0,0), eP_(0,1), current_pose_change(0,1), eP_(0,3), current_pose_change(0,2), eP_(0,5), eP_(0,6), eP_(0,7));// initial pose, final pose, initial vel, final vel, initial acc, final acc, initial time, final time
	results(1,0) = cal_end_point_tra_py -> fifth_order_traj_gen(current_pose_change(1,0), eP_(1,1), current_pose_change(1,1), eP_(1,3), current_pose_change(1,2), eP_(1,5), eP_(1,6), eP_(1,7));
	results(2,0) = cal_end_point_tra_pz -> fifth_order_traj_gen(current_pose_change(2,0), eP_(2,1), current_pose_change(2,1), eP_(2,3), current_pose_change(2,2), eP_(2,5), eP_(2,6), eP_(2,7));

	results(3,0) = cal_end_point_tra_alpha -> fifth_order_traj_gen(current_pose_change(3,0), eP_(3,1), current_pose_change(3,1), eP_(3,3), current_pose_change(3,2), eP_(3,5), eP_(3,6), eP_(3,7));
	results(4,0) = cal_end_point_tra_betta -> fifth_order_traj_gen(current_pose_change(4,0), eP_(4,1), current_pose_change(4,1), eP_(4,3), current_pose_change(4,2), eP_(4,5), eP_(4,6), eP_(4,7));
	results(5,0) = cal_end_point_tra_kamma -> fifth_order_traj_gen(current_pose_change(5,0), eP_(5,1), current_pose_change(5,1), eP_(5,3), current_pose_change(5,2), eP_(5,5), eP_(5,6), eP_(5,7));


	if( cal_end_point_tra_px->is_moving_traj || cal_end_point_tra_py->is_moving_traj || cal_end_point_tra_pz->is_moving_traj ||
			cal_end_point_tra_alpha ->is_moving_traj || cal_end_point_tra_betta ->is_moving_traj  || cal_end_point_tra_kamma ->is_moving_traj)
	{
		is_moving_check = true;
	}
	else
		is_moving_check = false;
}

double EndEffectorTraj::cal_one_joint_rad(Eigen::MatrixXd joint_)
{
	double result_one_joint_;

	if(cal_one_joint_traj_rad -> detect_change_final_value(joint_(0,1), joint_(0,3), joint_(0,7)))
	{
		current_one_joint_pose(0,0) = cal_one_joint_traj_rad -> current_pose;
		current_one_joint_pose(0,1) = cal_one_joint_traj_rad -> current_velocity;
		cal_one_joint_traj_rad -> current_time = 0;
	}
	result_one_joint_ = cal_one_joint_traj_rad -> fifth_order_traj_gen(current_one_joint_pose(0,0), joint_(0,1), current_one_joint_pose(0,1), joint_(0,3), joint_(0,4), joint_(0,5), joint_(0,6), joint_(0,7));
	is_moving_check = cal_one_joint_traj_rad->is_moving_traj;
	 return result_one_joint_;
}

Eigen::Matrix<double, 6, 1> EndEffectorTraj::get_traj_results()
{
  return results;
}
void EndEffectorTraj::set_control_time(double ctrl_time)
{
  cal_end_point_tra_px->control_time_ = ctrl_time;
  cal_end_point_tra_py->control_time_ = ctrl_time;
  cal_end_point_tra_pz->control_time_ = ctrl_time;

  cal_end_point_tra_alpha->control_time_ = ctrl_time;
  cal_end_point_tra_betta->control_time_ = ctrl_time;
  cal_end_point_tra_kamma->control_time_ = ctrl_time;

}
void EndEffectorTraj::stop_trajectory()
{
  cal_end_point_tra_px    ->stop_trajectory();
  cal_end_point_tra_py    ->stop_trajectory();
  cal_end_point_tra_pz    ->stop_trajectory();
  cal_end_point_tra_alpha ->stop_trajectory();
  cal_end_point_tra_betta ->stop_trajectory();
  cal_end_point_tra_kamma ->stop_trajectory();
}



