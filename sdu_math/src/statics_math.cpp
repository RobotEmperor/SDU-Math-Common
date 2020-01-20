/*
 * statics_math.cpp
 *
 *  Created on: Jan 17, 2020
 *      Author: yik
 */

#include "sdu_math/statics_math.h"


double calculate_cusum(double data, double allowable_slack_gain_k, double high_limit, double low_limit)
{
  static double detection = 0;
  static double s_h = 0;
  static double s_l = 0;
  static double pre_s_h = 0;
  static double pre_s_l = 0;
  static double target_value = 0;
  static double sum_h = 0;
  static double sum_l = 0;

  sum_h = pre_s_h + data - target_value - allowable_slack_gain_k;
  sum_l = pre_s_l + data - target_value + allowable_slack_gain_k;

  if(sum_h < 0)
    sum_h = 0;
  if(sum_l > 0)
    sum_l = 0;

  if(sum_h > high_limit)
    detection = 1;
  else if(sum_l < low_limit)
    detection = -1;
  else
    detection= 0;

  double temp_a;
  temp_a = sum_h;

  if(detection == 1 || detection == -1)
  {
    //target_value = data;
    pre_s_h, sum_h = 0;
    pre_s_l, sum_l = 0;
  }

  pre_s_h = sum_h;
  pre_s_l = sum_l;

  return detection;
}
