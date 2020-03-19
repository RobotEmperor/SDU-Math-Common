/*
 * end_point_to_rad_cal.h
 *
 *  Created on: Oct 25, 2017
 *      Author: robotemperor
 */

#ifndef SDU_MATH_SDU_MATH_INCLUDE_END_POINT_TO_RAD_CAL_H_
#define SDU_MATH_SDU_MATH_INCLUDE_END_POINT_TO_RAD_CAL_H_

#include "sdu_math/fifth_order_trajectory_generate.h"

class CalRad
{
public:
	CalRad();
	~CalRad();
	bool is_moving_check;

	void cal_end_point_to_rad(Eigen::MatrixXd eP_);
	double cal_one_joint_rad(Eigen::MatrixXd joint_);

	FifthOrderTrajectory *cal_end_point_tra_px;
	FifthOrderTrajectory *cal_end_point_tra_py;
	FifthOrderTrajectory *cal_end_point_tra_pz;

	FifthOrderTrajectory *cal_end_point_tra_alpha;
	FifthOrderTrajectory *cal_end_point_tra_betta;
	FifthOrderTrajectory *cal_end_point_tra_kamma;

	FifthOrderTrajectory *cal_one_joint_traj_rad;


	Eigen::MatrixXd current_pose_change;
	Eigen::MatrixXd current_one_joint_pose;

	Eigen::MatrixXd get_traj_results();

private:
	Eigen::MatrixXd results;
};











#endif /* SDU_MATH_SDU_MATH_INCLUDE_END_POINT_TO_RAD_CAL_H_ */
