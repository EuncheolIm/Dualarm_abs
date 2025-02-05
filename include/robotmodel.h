#pragma once
#ifndef __MODEL_H
#define __MODEL_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "custommath.h"

using namespace std;
using namespace Eigen;

class CModel
{
public:
	CModel();
	virtual ~CModel();

	void update_kinematics(VectorXd& q, VectorXd& qdot);
	void update_dynamics();
	void calculate_EE_positions_orientations();
	void calculate_EE_Jacobians();	
	void calculate_EE_velocity();

	RigidBodyDynamics::Model _model;
	RigidBodyDynamics::Model _model_co;
	MatrixXd _A; //inertia matrix
	VectorXd _g; //gravity force vector
	VectorXd _b; //Coriolis/centrifugal force vector
	VectorXd _bg; //Coriolis/centrifugal force vector + gravity force vector

	MatrixXd _J_left_hand, _J_right_hand;
	MatrixXd _J_left_hand_pos, _J_left_hand_ori, _J_right_hand_pos, _J_right_hand_ori;
	Vector3d _x_left_hand, _x_right_hand;
	Matrix3d _R_left_hand, _R_right_hand;
	VectorXd _xdot_left_hand, _xdot_right_hand;
	Vector3d _x_left_shoulder, _x_right_shoulder;

	VectorXd _max_joint_torque, _min_joint_torque, _max_joint_velocity, _min_joint_velocity, _max_joint_position, _min_joint_position, _max_ctrl_joint_torque, _min_ctrl_joint_torque;

	int _k, _k_co;//joint number

	MatrixXd _A_co; //inertia matrix
	VectorXd _g_co; //gravity force vector
	VectorXd _b_co; //Coriolis/centrifugal force vector
	VectorXd _bg_co; //Coriolis/centrifugal force vector + gravity force vector

	MatrixXd _J_left_lowerArm;     // 6 x 15
	MatrixXd _J_left_lowerArm_pos; // 3 x 15
	MatrixXd _J_left_lowerArm_ori; // 3 x 15

	Vector3d _x_left_lowerArm; // 3x1
	Vector3d _xdot_left_lowerArm; //3x1

	MatrixXd _J_left_lowerArm7;     // 6 x 7
	MatrixXd _J_left_lowerArm_pos7; // 3 x 7
	MatrixXd _J_left_lowerArm_ori7; // 3 x 7

	Vector3d _x_leftLower; // 3x1
	Vector3d _xdot_leftLower; //3x1
	MatrixXd _J_tmp7; //6x8
	VectorXd _q7, _qdot7; //8x1

	Eigen::Matrix<double, 15, 1> _avg_joint_position;
	Eigen::Matrix<double, 3, 1> _x_base;

private:
	void Initialize();
	void load_model();
	void set_robot_config();

	bool _bool_model_update, _bool_kinematics_update, _bool_dynamics_update, _bool_Jacobian_update;

	int _id_left_hand, _id_right_hand, _id_left_shoulder, _id_right_shoulder;
	int _id_left_lowerarm;


	VectorXd _q, _qdot;
	VectorXd _zero_vec_joint;

	Vector3d _position_local_task_left_hand;
	Vector3d _position_local_task_right_hand, _position_local_task_base;
	Vector3d _position_local_zerovec;
	Vector3d _position_local_task_left_hand_lower_arm;

	MatrixXd _J_tmp;	
	Matrix3d _global_rotate;	

	unsigned int base_id;
	
	Eigen::Matrix<double, 3, 3> _R_base;
	Eigen::Matrix<double, 6, 1> _x_base_6d;

};

#endif