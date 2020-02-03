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

int calculate_cusum(double data, double allowable_slack_gain_k, double high_limit, double low_limit);
double calculate_mean(double data);
Eigen::MatrixXd least_square_problem(Eigen::MatrixXd a, Eigen::MatrixXd b); // return X matrixXd






#endif /* SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_STATISTICCS_MATH_H_ */
