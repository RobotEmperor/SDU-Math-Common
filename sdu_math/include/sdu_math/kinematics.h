/*
 * kinematics.h
 *
 *  Created on: Jan 15, 2020
 *      Author: yik
 */

#ifndef SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_KINEMATICS_H_
#define SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_KINEMATICS_H_

#include <Eigen/Dense>
#include <vector>
#include <math.h>

class Kinematics
{

  public:
    Kinematics();
    ~Kinematics();

    std::vector<double> joint_positions;

    std::vector<Eigen::MatrixXd> transformation;

    Eigen::MatrixXd temp_data;

    Eigen::Matrix3d rotation_matrix_x(double radian);
    Eigen::Matrix3d rotation_matrix_y(double radian);
    Eigen::Matrix3d rotation_matrix_z(double radian);

    Eigen::Matrix4d transformation_result;

    void link_parameter(std::vector<double> theta);
    void calculate_forward_kinematics(int joint_id, std::vector<double> theta);


    double d_1, a_2, a_3, d_4, d_5, d_6;





};



#endif /* SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_KINEMATICS_H_ */
