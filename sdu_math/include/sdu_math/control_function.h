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

#include "sdu_math/control_function.h"
#include "sdu_math/end_point_to_rad_cal.h"

using namespace std;

class PID_function
{
public:
  PID_function(double dt, double max, double min, double kp, double kd, double ki, double threshold_max, double threshold_min);
  ~PID_function();
  double PID_calculate(double ref_value, double current_value, double input_threshold);
  double get_kp_gain();
  double get_ki_gain();
  double get_kd_gain();
  double get_final_output();
  void set_pid_gain(double p_gain,double i_gain,double d_gain);

private:
  EndEffectorTraj* gain_traj;

  Eigen::MatrixXd desired_values;

  double dt_;
  double pre_error_;
  double integral_;
  double kp_;
  double kd_;
  double ki_;
  double max_;
  double min_;
  double final_output;
  double threshold_max_;
  double threshold_min_;


};



#endif /* CONTROL_FUNCTION_H_ */
