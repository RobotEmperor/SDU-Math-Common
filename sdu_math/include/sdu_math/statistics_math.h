/*
 * statistics_math.h
 *
 *  Created on: Jan 17, 2020
 *      Author: yik
 */

#ifndef SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_STATISTICCS_MATH_H_
#define SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_STATISTICCS_MATH_H_


#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>

class Statistics
{
  public:
    Statistics();
    ~Statistics();
    int calculate_cusum(double data, double allowable_slack_gain_k, double high_limit, double low_limit);
    double calculate_mean(double data);
    double calculate_diff(double value, double control_time);
    void set_dimension(Eigen::MatrixXd data);
    Eigen::MatrixXd calculate_diff(Eigen::MatrixXd value, double control_time);
    Eigen::MatrixXd least_square_problem(Eigen::MatrixXd a, Eigen::MatrixXd b); // return X matrixXd

  private:
    Eigen::MatrixXd diff_value_;
    Eigen::MatrixXd previous_value_;



};
#endif /* SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_STATISTICCS_MATH_H_ */
