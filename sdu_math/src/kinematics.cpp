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


  //link parameter
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
