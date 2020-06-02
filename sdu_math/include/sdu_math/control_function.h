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

class PID_function
{
public:
  PID_function(double dt, double max, double min, double kp, double kd, double ki);
  ~PID_function();
  double PID_calculate(double ref_value, double current_value);
  double kp_;
  double kd_;
  double ki_;
  double max_;
  double min_;

private:
  double dt_;
  double pre_error_;
  double integral_;
};



#endif /* CONTROL_FUNCTION_H_ */
