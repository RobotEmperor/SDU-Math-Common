
/*
 * fifth_order_trajectory_generate.h
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */
#ifndef SDU_MATH_SDU_MATH_INCLUDE_FIFTH_ORDER_TRAJECTORY_GENERATE_H_
#define SDU_MATH_SDU_MATH_INCLUDE_FIFTH_ORDER_TRAJECTORY_GENERATE_H_

#include <Eigen/Dense>

class FifthOrderTrajectory

{
  public:
    FifthOrderTrajectory(double control_time);
    FifthOrderTrajectory();
    ~FifthOrderTrajectory();

    double fifth_order_traj_gen(double initial_value_, double final_value_,
        double initial_velocity_, double final_velocity_ ,
        double initial_acc, double final_acc,
        double initial_time_, double final_time_);

    bool   detect_change_final_value(double pose, double velocity_, double time_);
    double fifth_order_traj_gen_one_value(Eigen::MatrixXd joint_);

    void stop_trajectory();

    bool is_moving_traj;

    bool get_is_moving_traj();
    double get_current_time();
    double get_current_pose();
    double get_current_velocity();
    double get_current_acc();

    double get_control_time();

    void set_current_pose(double curr_pose);
    void set_current_time(double curr_time);
    void set_control_time(double ctrl_time);

  private:

    double initial_time;
    double initial_pose;
    double initial_velocity;
    double initial_acc;

    double current_time;
    double current_pose;
    double current_velocity;
    double current_acc;

    double final_time;
    double final_pose;
    double final_velocity;
    double final_acc;

    double control_time_;

    Eigen::Matrix<double, 6, 6>  time_mat;
    Eigen::Matrix<double, 6, 1>  conditions_mat;

    Eigen::MatrixXd position_coeff_;
    Eigen::Matrix<double, 6, 1> velocity_coeff_;
    Eigen::Matrix<double, 6, 1> acceleration_coeff_;
    Eigen::Matrix<double, 1, 6> time_variables_;
};

#endif /* SDU_MATH_SDU_MATH_INCLUDE_FIFTH_ORDER_TRAJECTORY_GENERATE_H_ */


