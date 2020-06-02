/*
 * control_function.cpp
 *
 *  Created on: Jun 2, 2020
 *      Author: yik
 */

#include "sdu_math/control_function.h"

/////// PID function /////////////////////////////////////////////////////////////////////////
PID_function::PID_function(double dt, double max, double min, double kp, double kd, double ki):
  dt_(dt),
  max_(max),
  min_(min),
  kp_(kp),
  ki_(ki),
  kd_(kd),
  pre_error_(0),
  integral_(0)
{
  gain_traj = std::make_shared<CalRad>();
  gain_traj->set_control_time(dt_);

  desired_values.resize(6,8);
  desired_values.fill(0);

  desired_values(0,7) = 2;
  desired_values(1,7) = 2;
  desired_values(2,7) = 2;

  for(int num=0;num<6;num++)
  {
    gain_traj->current_pose_change(num,0) = 0;
  }
  gain_traj->cal_end_point_tra_px->current_pose = 0;
  gain_traj->cal_end_point_tra_py->current_pose = 0;
  gain_traj->cal_end_point_tra_pz->current_pose = 0;
}
PID_function::~PID_function()
{
}
double PID_function::PID_calculate(double ref_value, double current_value)
{
    // calculate error
      double error = ref_value - current_value;

      // P gain
      double p_control_value = kp_ * error;

      // I gain
      integral_ += error * dt_;
      double i_control_value = ki_ * integral_;

      //D gain
      double derivate = (error - pre_error_)/dt_;
      double d_control_value = kd_ * derivate;

      // calculate control value

      double output = p_control_value + i_control_value + d_control_value;

      if(output > max_)
        output = max_;
      else if (output < min_)
        output = min_;

      pre_error_ = error;

      if(output < 0.0005 && output > -0.0005)
      output = 0;

      return output;
}
void PID_function::set_pid_gain(double p_gain,double i_gain,double d_gain)
{
  desired_values(0,1) = p_gain;
  desired_values(1,1) = i_gain;
  desired_values(2,1) = d_gain;

  gain_traj->cal_end_point_to_rad(desired_values);

  kp_ = gain_traj->get_traj_results()(0,0);
  ki_ = gain_traj->get_traj_results()(1,0);
  kd_ = gain_traj->get_traj_results()(2,0);
}
double PID_function::get_kp_gain()
{
  return kp_;
}
double PID_function::get_ki_gain()
{
  return ki_;
}
double PID_function::get_kd_gain()
{
  return kd_;
}


