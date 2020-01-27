# SDU-Math-Common
This is about SDU-Math-Common

## Introduction ##
SDU-Math-Common includes the following functions available: 

* kinematics

* statistics_math 

  Kinematics includes essential robotics math tool such as forward kinematics of UR10e, rotation matrix, transformation matrix. statistics_math is about basic statistics math like cusum method. 

## Dependencies ##
* [Eigen3] (http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Compatible Operating Systems ##
  Currently tested on:

* Ubuntu 18.04 (Bionic Beaver)

## Build ##
  Currently tested on:

* Cmake 

## Installation (Build Instruction) ##

  sdu_math

    git clone https://github.com/RobotEmperor/SDU-Math-Common.git
    cd sdu_math
    mkdir build
    cd build
    cmake ..
    make 
    sudo make install 
    
  These two libraries are installed in /usr/local/lib and the headers are in /usr/local/include
  
## Cmake Example ## 

    cmake_minimum_required(VERSION 3.5)
    project(your_project)
    
    link_directories(/usr/local/lib)
    add_executable(your_project main.cpp)
    target_link_libraries(your_project sdu_math)


## Classes and Functions ##

  ###kinematics###
  
  * calculate_forward_kinematics(int joint_id, std::vector<double> theta)
  
  It returns transformation_matrix according to current joint value. This function is used to transfrom tool's values from tool's frame to base frame because tool's values that ur_rtde outputs are relative to tool's frame and force torque sensor values that ur_rtde offers are relative to base frame.  
  
  * rotation_matix_x,y,z (double radian)
  
  It returns 3 X 3 rotation matrix by radian.
  
For example(kinematics),

    Kinematics ur10e_kinematics;
    ur10e_kinematics = new Kinematics; 
    
    // in the control loop
    
    ur10e_kinematics->temp_data(0,0) = tool_linear_acc_data[0]; // X
    ur10e_kinematics->temp_data(1,0) = tool_linear_acc_data[1]; // Y
    ur10e_kinematics->temp_data(2,0) = tool_linear_acc_data[2]; // Z
    
    ur10e_kinematics->calculate_forward_kinematics(6,ur10e_kinematics->joint_positions);
    ur10e_kinematics->temp_data = ur10e_kinematics->transformation_result*ur10e_kinematics->temp_data; // transformation
  
    //you can use  ur10e_kinematics->temp_data variables in your control algorithm.
  
    // program is ended 
    delete ur10e_kinematics;
    
    
    
  ###statistics###
  
  * double calculate_cusum(double data, double allowable_slack_gain_k, double high_limit, double low_limit)
  
  It offers collision detection based on force torque value. This function uses cusum method. 
  
For example(statistics),

    // in the control loop 
    
    fx_detection = calculate_cusum(data(0,0),fx_k,fx_high_limit,fx_low_limit); // decide k, limit
    
    //data(0,0) or a double varialbe input to this function. In here, raw force x data is entered.
    //fx_detection is 0, 1, -1. 0 is non contact, 1 is positive contact and -1 is negative contact. 
    
    
    
  
  

