/*
 * kinematics.cpp
 *
 *  Created on: Jan 15, 2020
 *      Author: yik
 */

#ifndef SDU_MATH_SDU_MATH_SRC_KINEMATICS_CPP_
#define SDU_MATH_SDU_MATH_SRC_KINEMATICS_CPP_

#include "sdu_math/kinematics.h"


Kinematics::Kinematics()
{
  temp_data.resize(4,1);
  temp_data.fill(0);
  transformation.resize(6);

  for(int num=0; num < 6; num++)
  {
    transformation[num].setIdentity(4,4);
  }

  jacobian_force_matrix.resize(6,6);
  jacobian_force_matrix.fill(0);

  force_matrix.resize(6,1);
  force_matrix.fill(0);

  torque_matrix.resize(6,1);
  torque_matrix.fill(0);

  z0.resize(3,1);
  z1.resize(3,1);
  z2.resize(3,1);
  z3.resize(3,1);
  z4.resize(3,1);
  z5.resize(3,1);

  z0.fill(0);
  z1.fill(0);
  z2.fill(0);
  z3.fill(0);
  z4.fill(0);
  z5.fill(0);


  //link parameter ur10e
  d_1 = 0.1807;
  a_2 = -0.6127;
  a_3 = -0.57155;
  d_4 = 0.17415;
  d_5 = 0.11985;
  d_6 = 0.11655;

}

Kinematics::~Kinematics()
{

}
void Kinematics::link_parameter(std::vector<double> theta)
{
}
void Kinematics::calculate_forward_kinematics(std::vector<double> theta)
{

  //  transformation[0] << cos(theta[0]), 0,  sin(theta[0]), 0,
  //    sin(theta[0]), 0, -cos(theta[0]), 0,
  //    0,1,0,d_1,
  //    0,0,0,1;
  //
  //  transformation[1] << cos(theta[1]), -sin(theta[1]), 0, a_2*cos(theta[1]),
  //    sin(theta[1]),  cos(theta[1]), 0, a_2*sin(theta[1]),
  //    0,0,1,0,
  //    0,0,0,1;
  //
  //  transformation[2] << cos(theta[2]), -sin(theta[2]), 0, a_3*cos(theta[2]),
  //    sin(theta[2]),  cos(theta[2]), 0, a_3*sin(theta[2]),
  //    0,0,1,0,
  //    0,0,0,1;
  //
  //  transformation[3] << cos(theta[3]), 0,  sin(theta[3]), 0,
  //    sin(theta[3]), 0, -cos(theta[3]), 0,
  //    0,1,0,d_4,
  //    0,0,0,1;
  //
  //  transformation[4] << cos(theta[4]), 0,-sin(theta[4]), 0,
  //    sin(theta[4]),0,  cos(theta[4]), 0,
  //    0,-1,0,d_5,
  //    0,0,0,1;
  //
  //  transformation[5] << cos(theta[5]), -sin(theta[5]), 0, 0,
  //    sin(theta[5]),  cos(theta[5]), 0, 0,
  //    0,0,1,d_6,
  //    0,0,0,1;
  //
  //
  transformation[0] = transformation_matrix(0,0,d_1,theta[0]);
  transformation[1] = transformation_matrix(M_PI/2,0,0,theta[1]);
  transformation[2] = transformation_matrix(0,a_2,0,theta[2]);

  transformation[3] = transformation_matrix(0,a_3,d_4,theta[3]);
  transformation[4] = transformation_matrix(M_PI/2,0,d_5,theta[4]);
  transformation[5] = transformation_matrix(-M_PI/2,0,d_6,theta[5]);

  transformation_result = transformation[0]*transformation[1]*transformation[2]*transformation[3]*transformation[4]*transformation[5];
}

void Kinematics::calculate_inverse_kinematics(std::vector<double> desired_pose)
{
  static Eigen::Matrix4d desired_tf;
  static Eigen::Matrix4d tf_1_4;
  static Eigen::Matrix4d tf_3_4;

  static Eigen::MatrixXd p_0_5;
  static Eigen::MatrixXd temp_d6;

  joint_results.clear();
  p_0_5.resize(4,1);
  p_0_5 << 0,0,0,1;
  temp_d6.resize(4,1);
  temp_d6 << 0,0,-d_6,1;

  static double theta_1 = 0;
  static double theta_2 = 0;
  static double theta_3 = 0;

  static double theta_4 = 0;
  static double theta_5 = 0;
  static double theta_6 = 0;

  static double x_kamma_ = 0;
  static double y_betta_ = 0;
  static double z_alpha_ = 0;

  static double pX_ = 0;
  static double pY_ = 0;
  static double pZ_ = 0;

  x_kamma_ = desired_pose[3];
  y_betta_ = desired_pose[4];
  z_alpha_ = desired_pose[5];
  pX_ = desired_pose[0];
  pY_ = desired_pose[1];
  pZ_ = desired_pose[2];

  // Euler angle initialize
  desired_tf(0,0) = cos(z_alpha_)*cos(y_betta_); //r11
  desired_tf(0,1) = cos(z_alpha_)*sin(y_betta_)*sin(x_kamma_) - sin(z_alpha_)*cos(x_kamma_);// r12
  desired_tf(0,2) = cos(z_alpha_)*sin(y_betta_)*cos(x_kamma_) + sin(z_alpha_)*sin(x_kamma_); //r13
  desired_tf(0,3) = pX_;

  desired_tf(1,0) = sin(z_alpha_)*cos(y_betta_); // r21
  desired_tf(1,1) = sin(z_alpha_)*sin(y_betta_)*sin(x_kamma_) + cos(z_alpha_)*cos(x_kamma_); //r22
  desired_tf(1,2) = sin(z_alpha_)*sin(y_betta_)*cos(x_kamma_) - cos(z_alpha_)*sin(x_kamma_);  // r23
  desired_tf(1,3) = pY_;

  desired_tf(2,0) = -sin(y_betta_); // r31
  desired_tf(2,1) = cos(y_betta_)*sin(x_kamma_);  //r32
  desired_tf(2,2) = cos(y_betta_)*cos(x_kamma_);  // r33
  desired_tf(2,3) = pZ_;

  desired_tf(3,0) = 0;
  desired_tf(3,1) = 0;
  desired_tf(3,2) = 0;
  desired_tf(3,3) = 1;

  p_0_5 = desired_tf * temp_d6;

  theta_1 = atan2(p_0_5(1,0),p_0_5(0,0)) - acos(d_4/sqrt(pow(p_0_5(1,0),2)+ pow(p_0_5(0,0),2))) + M_PI/2; // two solution +- acos

  theta_5 = acos((desired_tf(0,3)*sin(theta_1) - desired_tf(1,3)*cos(theta_1) - d_4)/d_6); // // two solution +- acos

  static double temp_y = 0;
  static double temp_x = 0;

  if(desired_tf.determinant() == 0 || theta_5 == 0)
    return;

  temp_y = (-desired_tf.inverse()(1,0)*sin(theta_1) + desired_tf.inverse()(1,1)*cos(theta_1) )/sin(theta_5);
  temp_x = (desired_tf.inverse()(0,0)*sin(theta_1) - desired_tf.inverse()(0,1)*cos(theta_1) )/sin(theta_5);

  theta_6 = atan2(temp_y, temp_x);

  if(transformation_matrix(0,0,d_1,theta_1).determinant() == 0 || (transformation_matrix(M_PI/2,0,d_5,theta_5) * transformation_matrix(-M_PI/2,0,d_6,theta_6)).determinant() == 0)
    return;

  tf_1_4 = transformation_matrix(0,0,d_1,theta_1).inverse() * desired_tf *(transformation_matrix(M_PI/2,0,d_5,theta_5) * transformation_matrix(-M_PI/2,0,d_6,theta_6)).inverse();

  theta_3 = - acos((pow(sqrt(pow(tf_1_4(2,3),2) + pow(tf_1_4(0,3),2)),2) - pow(a_2,2)- pow(a_3,2)) / (2*a_2*a_3));// two solution + - acos

  theta_2 = atan2(-tf_1_4(2,3), -tf_1_4(0,3)) - asin((-a_3*sin(theta_3))/sqrt(pow(tf_1_4(2,3),2) + pow(tf_1_4(0,3),2)));

  if((transformation_matrix(0,0,d_1,theta_1)*transformation_matrix(M_PI/2,0,0,theta_2)*transformation_matrix(0,a_2,0,theta_3)).determinant() == 0)
    return;

  tf_3_4 = (transformation_matrix(0,0,d_1,theta_1)*transformation_matrix(M_PI/2,0,0,theta_2)*transformation_matrix(0,a_2,0,theta_3)).inverse() * desired_tf * (transformation_matrix(M_PI/2,0,d_5,theta_5) * transformation_matrix(-M_PI/2,0,d_6,theta_6)).inverse();

  theta_4 = atan2(tf_3_4(1,0),tf_3_4(0,0));

  joint_results.push_back(theta_1);
  joint_results.push_back(theta_2);
  joint_results.push_back(theta_3);
  joint_results.push_back(theta_4);
  joint_results.push_back(theta_5);
  joint_results.push_back(theta_6);
}
void Kinematics::calculate_jacobian(std::vector<double> theta)
{
  double q0,q1,q2,q3,q4,q5;
  q0 = theta[0];
  q1 = theta[1];
  q2 = theta[2];

  q3 = theta[3];
  q4 = theta[4];
  q5 = theta[5];

  z0 << 0,
      0,
      1;



  z1 << sin(q0),
      -cos(q0),
      0;


  z2 << sin(q0),
      -cos(q0),
      0;


  z3 << sin(q0),
      -cos(q0),
      0;


  z4 << cos(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)),
      cos(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - sin(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)),
      sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2));


  z5 << cos(q4)*sin(q0) - sin(q4)*(cos(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1))),
      sin(q4)*(cos(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + sin(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))) - cos(q0)*cos(q4),
      -sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)));

  jacobian_force_matrix << d_4*cos(q0) - d_5*(cos(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - sin(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0))) + d_6*(cos(q0)*cos(q4) - sin(q4)*(cos(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + sin(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)))) - a_2*cos(q1)*sin(q0) - a_3*cos(q1)*cos(q2)*sin(q0) + a_3*sin(q0)*sin(q1)*sin(q2), d_5*(cos(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1))) - a_2*cos(q0)*sin(q1) + d_6*sin(q4)*(cos(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2))) - a_3*cos(q0)*cos(q1)*sin(q2) - a_3*cos(q0)*cos(q2)*sin(q1), d_5*(cos(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1))) + d_6*sin(q4)*(cos(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2))) - a_3*cos(q0)*cos(q1)*sin(q2) - a_3*cos(q0)*cos(q2)*sin(q1), d_5*(cos(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1))) + d_6*sin(q4)*(cos(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2))), -d_6*(sin(q0)*sin(q4) + cos(q4)*(cos(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)))), 0,
      d_5*(cos(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2))) + d_4*sin(q0) + d_6*(cos(q4)*sin(q0) - sin(q4)*(cos(q3)*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)))) + a_2*cos(q0)*cos(q1) + a_3*cos(q0)*cos(q1)*cos(q2) - a_3*cos(q0)*sin(q1)*sin(q2), d_6*sin(q4)*(cos(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - sin(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0))) - d_5*(cos(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + sin(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))) - a_2*sin(q0)*sin(q1) - a_3*cos(q1)*sin(q0)*sin(q2) - a_3*cos(q2)*sin(q0)*sin(q1), d_6*sin(q4)*(cos(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - sin(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0))) - d_5*(cos(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + sin(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))) - a_3*cos(q1)*sin(q0)*sin(q2) - a_3*cos(q2)*sin(q0)*sin(q1), d_6*sin(q4)*(cos(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - sin(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0))) - d_5*(cos(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + sin(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))),  d_6*(cos(q0)*sin(q4) + cos(q4)*(cos(q3)*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + sin(q3)*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)))), 0,
      0,                                                                                         a_2*cos(q1) + d_5*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + a_3*cos(q1)*cos(q2) - a_3*sin(q1)*sin(q2) - d_6*sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))),                                                                                 d_5*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) + a_3*cos(q1)*cos(q2) - a_3*sin(q1)*sin(q2) - d_6*sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))),                                                                 d_5*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - d_6*sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))),                                                     -d_6*cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))), 0,
      z0(0,0),z1(0,0),z2(0,0),z3(0,0),z4(0,0),z5(0,0),
      z0(1,0),z1(1,0),z2(1,0),z3(1,0),z4(1,0),z5(1,0),
      z0(2,0),z1(2,0),z2(2,0),z3(2,0),z4(2,0),z5(2,0);


}
void Kinematics::calculate_end_effector_force(std::vector<double> torque)
{
  torque_matrix << torque[0],
      torque[1],
      torque[2],
      torque[3],
      torque[4],
      torque[5];


  force_matrix = (jacobian_force_matrix.transpose()).inverse() * torque_matrix;
}

void Kinematics::set_joint_positions(std::vector<double> input_joint_position)
{
  joint_positions = input_joint_position;
}
void Kinematics::set_dh_parameter(double d1,double a2,double a3,double d4,double d5,double d6)
{
  d_1 = d1;
  a_2 = a2;
  a_3 = a3;
  d_4 = d4;
  d_5 = d5;
  d_6 = d6;
}
Eigen::MatrixXd Kinematics::tf_base_to_tool(Eigen::MatrixXd input_data)
{
  return transformation_result*input_data;
}

Eigen::Matrix3d Kinematics::rotation_matrix_x(double radian)
{
  Eigen::Matrix3d r;

  r   << 1, cos(radian), -sin(radian),
      0, sin(radian),  cos(radian),
      0,           0,            0;

  return r;


}
Eigen::Matrix3d Kinematics::rotation_matrix_y(double radian)
{
  Eigen::Matrix3d r;

  r   <<  cos(radian), 0,  sin(radian),
           0, 1,  0,
      -sin(radian), 0,     cos(radian);

  return r;


}
Eigen::Matrix3d Kinematics::rotation_matrix_z(double radian)
{
  Eigen::Matrix3d r;

  r   << cos(radian), -sin(radian), 0,
      sin(radian),  cos(radian), 0,
      0,            0, 1;

  return r;
}
Eigen::Matrix4d Kinematics::transformation_matrix(double alpha, double a, double d, double theta)
{
  Eigen::Matrix4d t;

  t << cos(theta), -sin(theta), 0 , a,
      sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
      sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
      0,0,0,1;

  return t;
}
Eigen::MatrixXd Kinematics::get_axis_to_euler_angle(double val_x, double val_y, double val_z)
{
  static double x = 0;
  static double y = 0;
  static double z = 0;
  static double qx = 0;
  static double qy = 0;
  static double qz = 0;
  static double qw = 0;
  static double magnitude = 0;

  Eigen::MatrixXd euler_angle; // defines roll pitch yaw
  euler_angle.resize(3,1);
  euler_angle.fill(0);

  magnitude = sqrt(pow(val_x,2) + pow(val_y,2) + pow(val_z,2));


  if(magnitude == 0)
  {
    //printf("magnitude is wrong!! angle returns the previous angle. \n\n");
    return euler_angle;
  }

  x = val_x/magnitude;
  y = val_y/magnitude;
  z = val_z/magnitude;

  qx = x * sin(magnitude/2);
  qy = y * sin(magnitude/2);
  qz = z * sin(magnitude/2);
  qw = cos(magnitude/2);

  euler_angle(2,0) = atan(2.0 * (qx*qy + qz*qw)/(qx*qx - qy*qy - qz*qz + qw*qw));
  euler_angle(0,0) = atan(2.0 * (qy*qz + qx*qw)/(-qx*qx - qy*qy + qz*qz + qw*qw));
  euler_angle(1,0) = asin(-2.0 * (qx*qz - qy*qw));

  return euler_angle;
}
Eigen::Matrix3d Kinematics::get_axis_to_rotation_matrix(double val_x, double val_y, double val_z)
{
  static double x = 0;
  static double y = 0;
  static double z = 0;
  static double c = 0;
  static double s = 0;
  static double t = 0;
  static double magnitude = 0;

  Eigen::Matrix3d r;
  r.fill(0);

  magnitude = sqrt(pow(val_x,2) + pow(val_y,2) + pow(val_z,2));

  if(magnitude == 0)
  {
    //printf("magnitude is wrong!! angle returns the previous angle. \n\n");
    return r;
  }
  x = val_x/magnitude;
  y = val_y/magnitude;
  z = val_z/magnitude;

  c = cos(magnitude);
  s = sin(magnitude);
  t = 1-c;

  r << t*x*x + c, t*x*y - z*s, t*x*z + y*s,
      t*x*y + z*s, t*y*y + c, t*y*z - x*s,
      t*x*z - y*s, t*y*z + x*s, t*z*z + c;

  return r;
}
Eigen::Matrix4d Kinematics::get_tf_base_to_tool()
{
  Eigen::Matrix4d t;

  t = transformation_result;

  return t;
}
Eigen::MatrixXd Kinematics::get_rotation_base_to_tool()
{
  return transformation_result.block<2,2>(0,0);
}
std::vector<double> Kinematics::get_ik_joint_results()
{
  std::vector<double> j;
  j.resize(6);

  j = joint_results;

  return j;
}
Eigen::Matrix4d Kinematics::desired_rotation_matrix(double roll, double pitch, double yaw)
{
  static Eigen::Matrix4d m_roll;
  static Eigen::Matrix4d m_pitch;
  static Eigen::Matrix4d m_yaw;

  m_roll << 1, cos(roll), -sin(roll), 0,
            0, sin(roll),  cos(roll), 0,
            0,         0,          0, 0,
            0,         0,          0, 1;

  m_yaw   << cos(yaw), -sin(yaw), 0,0,
             sin(yaw),  cos(yaw), 0,0,
             0,                0, 1,0,
             0,                0, 0,1;

  m_pitch  <<cos(pitch), 0,  sin(pitch),0,
                      0, 1,           0,0,
            -sin(pitch), 0,  cos(pitch),0,
                      0, 0,           0,1;

  return m_yaw*m_pitch*m_roll;
}

Eigen::Matrix4d Kinematics::desired_transformation_matrix(double x, double y, double z, double roll, double pitch, double yaw)
{
  static Eigen::Matrix4d tf;

  tf = desired_rotation_matrix(roll, pitch, yaw);
  tf(0,3) = x;
  tf(1,3) = y;
  tf(2,3) = z;

  return tf;
}

#endif /* SDU_MATH_SDU_MATH_SRC_KINEMATICS_CPP_ */
