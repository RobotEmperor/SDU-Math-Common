/*
 * statistics_math.cpp
 *
 *  Created on: Jan 17, 2020
 *      Author: yik
 */

#include "sdu_math/statistics_math.h"


int calculate_cusum(double data, double allowable_slack_gain_k, double high_limit, double low_limit)
{
  static int detection = 0;
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


Eigen::MatrixXd least_square_problem(Eigen::MatrixXd a, Eigen::MatrixXd b)
{
  static Eigen::MatrixXd x;
  x.resize(a.rows(),b.rows());
  //x.fill(0);

  if((a.transpose()*a).determinant() != 0)
  {
    x = (a.transpose()*a).inverse()*a.transpose()*b;
  }
  else
  {
    printf("This matrix is not invertible \n");
  }

  return x;
  // return X matrixXd
}
double calculate_mean(double data)
{
  static double pre_result_mean = 0;
  static double number = 0;
  static double result_mean = 0;

  number ++;

  result_mean = ((pre_result_mean)*(number-1) + data)/number;

  pre_result_mean = result_mean;

  if(number > 1000)
  {
    number = 0;
  }

  return result_mean;
}




////////////////////////////////////////////////////////////////////////////




