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

#define DEGREE2RADIAN M_PI/180.0
#define RADIAN2DEGREE  180.0/M_PI

class Kinematics
{

  public:
    Kinematics();
    ~Kinematics();

    void link_parameter(std::vector<double> theta);
    void calculate_forward_kinematics(std::vector<double> theta);
    void calculate_inverse_kinematics(std::vector<double> desired_pose);
    void calculate_jacobian(std::vector<double> theta);
    void calculate_end_effector_force(std::vector<double> torque);

    void set_joint_positions(std::vector<double> input_joint_position);
    void set_dh_parameter(double d1,double a2,double a3,double d4,double d5,double d6);

    Eigen::Matrix4d get_tf_base_to_tool();
    Eigen::MatrixXd get_rotation_base_to_tool();
    Eigen::MatrixXd get_axis_to_euler_angle(double val_x, double val_y, double val_z);
    Eigen::Matrix3d get_axis_to_rotation_matrix(double val_x, double val_y, double val_z);
    std::vector<double> get_ik_joint_results();

    Eigen::MatrixXd tf_base_to_tool(Eigen::MatrixXd input_data);
    Eigen::Matrix4d transformation_matrix(double alpha, double a, double d, double theta);
    Eigen::Matrix4d desired_transformation_matrix(double x, double y, double z, double roll, double pitch, double yaw);
    Eigen::Matrix4d desired_rotation_matrix(double roll, double pitch, double yaw);

  private:
    double d_1, a_2, a_3, d_4, d_5, d_6;

    std::vector<double> joint_positions;
    std::vector<double> joint_results;

    std::vector<Eigen::Matrix4d> transformation;

    Eigen::MatrixXd temp_data;

    Eigen::Matrix3d rotation_matrix_x(double radian);
    Eigen::Matrix3d rotation_matrix_y(double radian);
    Eigen::Matrix3d rotation_matrix_z(double radian);

    Eigen::Matrix4d transformation_result;

    Eigen::MatrixXd jacobian_force_matrix;
    Eigen::MatrixXd force_matrix;

    Eigen::MatrixXd torque_matrix;

    Eigen::MatrixXd z0,z1,z2,z3,z4,z5;

};
#endif /* SDU_MATH_SDU_MATH_INCLUDE_SDU_MATH_KINEMATICS_H_ */
