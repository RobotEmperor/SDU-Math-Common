/*
 * control_function.h
 *
 *  Created on: Jun 2, 2020
 *      Author: yik
 */

#ifndef CONTROL_FUNCTION_H_
#define CONTROL_FUNCTION_H_

#include <stdio.h>
#include <math.h>
#include <memory>

#include "control_function.h"
#include "end_point_to_rad_cal.h"

using namespace std;

class PID_function
{
public:
  PID_function(double dt, double max, double min, double kp, double kd, double ki, double threshold_max, double threshold_min, double time);
  ~PID_function();
  double PID_calculate(double ref_value, double current_value, double input_threshold);
  double get_kp_gain();
  double get_ki_gain();
  double get_kd_gain();
  double get_final_output();
  double get_error();
  void set_pid_gain(double p_gain,double i_gain,double d_gain);
  void set_smooth_gain_time(double time);

private:
  EndEffectorTraj* gain_traj;

  Eigen::Matrix<double, 6, 8>  desired_values;

  double dt_;
  double error_;
  double pre_error_;
  double integral_;
  double kp_;
  double kd_;
  double ki_;
  double max_;
  double min_;
  double final_output_;
  double threshold_max_;
  double threshold_min_;
  double time_;


};



#endif /* CONTROL_FUNCTION_H_ */
