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
void Kinematics::calculate_forward_kinematics(int joint_id, std::vector<double> theta)
{

  transformation[0] << cos(theta[0]), 0,  sin(theta[0]), 0,
                       sin(theta[0]), 0, -cos(theta[0]), 0,
                       0,1,0,d_1,
                       0,0,0,1;

  transformation[1] << cos(theta[1]), -sin(theta[1]), 0, a_2*cos(theta[1]),
                       sin(theta[1]),  cos(theta[1]), 0, a_2*sin(theta[1]),
                       0,0,1,0,
                       0,0,0,1;

  transformation[2] << cos(theta[2]), -sin(theta[2]), 0, a_3*cos(theta[2]),
                       sin(theta[2]),  cos(theta[2]), 0, a_3*sin(theta[2]),
                       0,0,1,0,
                       0,0,0,1;

  transformation[3] << cos(theta[3]), 0,  sin(theta[3]), 0,
                       sin(theta[3]), 0, -cos(theta[3]), 0,
                       0,1,0,d_4,
                       0,0,0,1;

  transformation[4] << cos(theta[4]), 0,-sin(theta[4]), 0,
                       sin(theta[4]),0,  cos(theta[4]), 0,
                       0,-1,0,d_5,
                       0,0,0,1;

  transformation[5] << cos(theta[5]), -sin(theta[5]), 0, 0,
                       sin(theta[5]),  cos(theta[5]), 0, 0,
                       0,0,1,d_6,
                       0,0,0,1;


  transformation_result = transformation[0]*transformation[1]*transformation[2]*transformation[3]*transformation[4]*transformation[5];


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
      -sin(radian), 1,  cos(radian),
      0, 0,            0;

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


#endif /* SDU_MATH_SDU_MATH_SRC_KINEMATICS_CPP_ */
