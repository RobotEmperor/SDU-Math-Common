/*
 * fifth_order_trajectory_generate.cpp
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */


#include <sdu_math/fifth_order_trajectory_generate.h>

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

FifthOrderTrajectory::FifthOrderTrajectory(double control_time)
{

  control_time_ = control_time;

  current_time   = 0;
  is_moving_traj = false;

  initial_time     = 0;
  initial_pose     = 0;
  initial_velocity = 0;
  initial_acc      = 0;

  current_time     = 0;
  current_pose     = 0;
  current_velocity = 0;
  current_acc      = 0;

  final_time       = 0;
  final_pose       = 0;
  final_velocity   = 0;
  final_acc        = 0;

  position_coeff_.resize(6,1);

  position_coeff_.fill(0);
  velocity_coeff_.fill(0);
  acceleration_coeff_.fill(0);
  time_variables_.fill(0);
}
FifthOrderTrajectory::FifthOrderTrajectory()
{
  control_time_ = 0;

  current_time   = 0;
  is_moving_traj = false;

  initial_time     = 0;
  initial_pose     = 0;
  initial_velocity = 0;
  initial_acc      = 0;

  current_time     = 0;
  current_pose     = 0;
  current_velocity = 0;
  current_acc      = 0;

  final_time       = 0;
  final_pose       = 0;
  final_velocity   = 0;
  final_acc        = 0;

  position_coeff_.resize(6,1);

  position_coeff_.fill(0);
  velocity_coeff_.fill(0);
  acceleration_coeff_.fill(0);
  time_variables_.fill(0);
}

FifthOrderTrajectory::~FifthOrderTrajectory()
{

}

bool FifthOrderTrajectory::detect_change_final_value(double pose_, double velocity_, double time_)
{
  if(pose_ != final_pose || velocity_ != final_velocity || time_ != final_time  )
  {
    final_pose = pose_;
    final_velocity = velocity_;
    final_time = time_;
    current_time = 0;
    return true;
  }
  else
    return false;

}

double FifthOrderTrajectory::fifth_order_traj_gen(double initial_value_, double final_value_,
    double initial_velocity_, double final_velocity_ ,
    double initial_acc_, double final_acc_,
    double initial_time_, double final_time_)
{
  if(final_time_ == 0)
  {
    is_moving_traj = false;
    return current_pose;
  }

  if(current_time == 0)
  {
    final_pose = final_value_;
    final_velocity = final_velocity_;
    final_acc = final_acc_;
    final_time = final_time_;

    initial_value_    = current_pose;
    initial_velocity_ = current_velocity;
    initial_acc_      = current_acc;

    static Eigen::Matrix<double, 6, 6>  time_mat;
    static Eigen::Matrix<double, 6, 1>  conditions_mat;
    time_mat.fill(0);
    conditions_mat.fill(0);

    time_mat <<  pow(initial_time_, 5),      pow(initial_time_, 4),     pow(initial_time_, 3),  pow(initial_time_, 2),  initial_time_, 1.0,
        5.0*pow(initial_time_, 4),  4.0*pow(initial_time_, 3), 3.0*pow(initial_time_, 2),        2.0*initial_time_,            1.0, 0.0,
        20.0*pow(initial_time_, 3), 12.0*pow(initial_time_, 2),           6.0*initial_time_,                      2.0,            0.0, 0.0,
        pow(final_time_, 5),        pow(final_time_, 4),       pow(final_time_, 3),    pow(final_time_, 2),    final_time_, 1.0,
        5.0*pow(final_time_, 4),    4.0*pow(final_time_, 3),   3.0*pow(final_time_, 2),          2.0*final_time_,            1.0, 0.0,
        20.0*pow(final_time_, 3),   12.0*pow(final_time_, 2),             6.0*final_time_,                      2.0,            0.0, 0.0;
    conditions_mat << initial_value_,
        initial_velocity_,
        initial_acc_,
        final_value_,
        final_velocity_,
        final_acc_;

    if(time_mat.determinant() == 0)
      return current_pose;

    position_coeff_ = time_mat.inverse() * conditions_mat;

    velocity_coeff_ <<0.0,
        5.0*position_coeff_.coeff(0,0),
        4.0*position_coeff_.coeff(1,0),
        3.0*position_coeff_.coeff(2,0),
        2.0*position_coeff_.coeff(3,0),
        1.0*position_coeff_.coeff(4,0);
    acceleration_coeff_ <<0.0,
        0.0,
        20.0*position_coeff_.coeff(0,0),
        12.0*position_coeff_.coeff(1,0),
        6.0*position_coeff_.coeff(2,0),
        2.0*position_coeff_.coeff(3,0);


    is_moving_traj = true;
  }

  if(current_time <= final_time_)
  {
    current_time = current_time + control_time_;
    time_variables_ << pow(current_time, 5), pow(current_time, 4), pow(current_time, 3), pow(current_time, 2), current_time, 1.0;
    current_pose     = (time_variables_ * position_coeff_).coeff(0,0);
    current_velocity = (time_variables_ * velocity_coeff_).coeff(0,0);
    current_acc      = (time_variables_ * acceleration_coeff_).coeff(0,0);

    is_moving_traj = true;

    return current_pose;
  }
  else
  {
    is_moving_traj = false;
    return current_pose;
  }
}

double FifthOrderTrajectory::fifth_order_traj_gen_one_value(Eigen::MatrixXd joint_)
{
  double result_one_joint_;

  if(detect_change_final_value(joint_(0,1), joint_(0,3), joint_(0,7)))
  {
    current_time = 0;
  }
  result_one_joint_ = fifth_order_traj_gen(current_pose, joint_(0,1), current_velocity, joint_(0,3), joint_(0,4), joint_(0,5), joint_(0,6), joint_(0,7));

  return result_one_joint_;
}

void FifthOrderTrajectory::stop_trajectory()
{
  current_velocity = 0;
  current_acc = 0;
}
bool FifthOrderTrajectory::get_is_moving_traj()
{
  return is_moving_traj;
}
double FifthOrderTrajectory::get_current_time()
{
  return current_time;
}
double FifthOrderTrajectory::get_current_pose()
{
  return current_pose;
}
double FifthOrderTrajectory::get_current_velocity()
{
  return current_velocity;
}
double FifthOrderTrajectory::get_current_acc()
{
  return current_acc;
}
void FifthOrderTrajectory::set_current_time(double curr_time)
{
  current_time = curr_time;
}
void FifthOrderTrajectory::set_control_time(double ctrl_time)
{
  control_time_ = ctrl_time;
}
void FifthOrderTrajectory::set_current_pose(double curr_pose)
{
  current_pose = curr_pose;
}
