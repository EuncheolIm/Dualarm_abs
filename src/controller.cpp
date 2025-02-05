#include "controller.h"
#include <chrono>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <iostream>
#include <experimental/filesystem>

CController::CController(int JDOF)
{
	_dofj = JDOF;
	Initialize();
	
}

CController::~CController()
{
}

// void CController::read(double t, double* q, double* qdot)
void CController::read(double t, double* q, double* qdot, double* object_pose)
{	
	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _dofj; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i]; //from simulator

		_pre_qdot(i) = _qdot(i);
	}
	for(int i=0; i<7; i++)
	{
		obj_pose(i) = object_pose[i];
	}

    //  contact force
	// CT_Force.block<6, 1>(0, 0) = FT_left;
	// CT_Force.block<6, 1>(6, 0) = FT_right;
}

void CController::write(double* torque) // torque
{
	for (int i = 0; i < 15; i++)
	{
		torque[i] = _torque(i);
	} 
}

void CController::reset_target(double motion_time, VectorXd target_joint_position)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position;
	_qdot_goal.setZero();
}

void CController::reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_left_hand.head(3) = target_pos_lh;
	_x_goal_left_hand.tail(3) = target_ori_lh;
	_xdot_goal_left_hand.setZero();
	_x_goal_right_hand.head(3) = target_pos_rh;
	_x_goal_right_hand.tail(3) = target_ori_rh;
	_xdot_goal_right_hand.setZero();
}

void CController::motionPlan()
{
	_time_plan(1) = 2.0; //move home position
	_time_plan(2) = 5.0; //move home position
	
	if (_bool_plan(_cnt_plan) == 1)
	{
		_cnt_plan = _cnt_plan + 1;
		if (_cnt_plan == 1)
		{	
			_q_home.setZero();
			_q_home(1) = 10 * DEG2RAD;//-q_rand_val[10];//45 * DEG2RAD;
			_q_home(2) = 10 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 
			_q_home(4) = 50 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 

			_q_home(8) = -10 * DEG2RAD;//-q_rand_val[10];//45 * DEG2RAD;
			_q_home(9) = -10 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 
			_q_home(11) = -50 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 

			reset_target(_time_plan(_cnt_plan), _q_home);
		}
	
		// else if (_cnt_plan == 2)
		// {
		// 	// _pos_goal_left_hand(0) = _x_left_hand(0) + 0.1;
		// 	// _pos_goal_left_hand(1) = _x_left_hand(1);
		// 	// _pos_goal_left_hand(2) = _x_left_hand(2) + 0.3;
		// 	// _rpy_goal_left_hand(0) = 90.0 * DEG2RAD;
		// 	// _rpy_goal_left_hand(1) = -90.0 * DEG2RAD;
		// 	// _rpy_goal_left_hand(2) = 0 * DEG2RAD;

		// 	// _pos_goal_right_hand(0) = _x_right_hand(0);
		// 	// _pos_goal_right_hand(1) = _x_right_hand(1);
		// 	// _pos_goal_right_hand(2) = _x_right_hand(2);
		// 	// _rpy_goal_right_hand(0) = _x_right_hand(3);
		// 	// _rpy_goal_right_hand(1) = _x_right_hand(4);
		// 	// _rpy_goal_right_hand(2) = _x_right_hand(5);

		// 	// reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// }
		// else if (_cnt_plan == 4)
		// {
		// 		_pos_goal_left_hand(0) = _x_left_hand(0) - 0.09;
		// 		_pos_goal_left_hand(1) = _x_left_hand(1);// - 0.03;
		// 		_pos_goal_left_hand(2) = _x_left_hand(2);
		// 		_rpy_goal_left_hand(0) = _x_left_hand(3)+35 * DEG2RAD;
		// 		_rpy_goal_left_hand(1) = -90 * DEG2RAD;//_x_left_hand(4);
		// 		_rpy_goal_left_hand(2) = _x_left_hand(5);

		// 		_pos_goal_right_hand(0) = _x_right_hand(0)-0.09;
		// 		_pos_goal_right_hand(1) = _x_right_hand(1);//+0.03;
		// 		_pos_goal_right_hand(2) = _x_right_hand(2);
		// 		_rpy_goal_right_hand(0) = _x_right_hand(3)-35 * DEG2RAD;//0;//080 * DEG2RAD;
		// 		_rpy_goal_right_hand(1) = -90 * DEG2RAD;//_x_right_hand(4);//-180 * DEG2RAD;
		// 		_rpy_goal_right_hand(2) = _x_right_hand(5);//90 * DEG2RAD;
		// 		reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// }
		// else if (_cnt_plan == 5)
		// {
		// 		_pos_goal_left_hand(0) = _x_left_hand(0) - 0.04;
		// 		_pos_goal_left_hand(1) = _x_left_hand(1)- 0.03;
		// 		_pos_goal_left_hand(2) = _x_left_hand(2);
		// 		_rpy_goal_left_hand(0) = _x_left_hand(3)+35 * DEG2RAD;
		// 		_rpy_goal_left_hand(1) = -90 * DEG2RAD;//_x_left_hand(4);
		// 		_rpy_goal_left_hand(2) = _x_left_hand(5);

		// 		_pos_goal_right_hand(0) = _x_right_hand(0)-0.04;
		// 		_pos_goal_right_hand(1) = _x_right_hand(1)+0.03;
		// 		_pos_goal_right_hand(2) = _x_right_hand(2);
		// 		_rpy_goal_right_hand(0) = _x_right_hand(3)-35 * DEG2RAD;//0;//080 * DEG2RAD;
		// 		_rpy_goal_right_hand(1) = -90 * DEG2RAD;//_x_right_hand(4);//-180 * DEG2RAD;
		// 		_rpy_goal_right_hand(2) = _x_right_hand(5);//90 * DEG2RAD;
		// 		reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// }
		// else if (_cnt_plan == 6)
		// {
		// 		_pos_goal_left_hand(0) = _x_left_hand(0) + 0.04;
		// 		_pos_goal_left_hand(1) = _x_left_hand(1) + 0.03;
		// 		_pos_goal_left_hand(2) = _x_left_hand(2);
		// 		_rpy_goal_left_hand(0) = _x_left_hand(3)-35 * DEG2RAD;
		// 		_rpy_goal_left_hand(1) = -90 * DEG2RAD;//_x_left_hand(4);
		// 		_rpy_goal_left_hand(2) = _x_left_hand(5);

		// 		_pos_goal_right_hand(0) = _x_right_hand(0)+0.04;
		// 		_pos_goal_right_hand(1) = _x_right_hand(1)-0.03;
		// 		_pos_goal_right_hand(2) = _x_right_hand(2);
		// 		_rpy_goal_right_hand(0) = _x_right_hand(3)+35 * DEG2RAD;//0;//080 * DEG2RAD;
		// 		_rpy_goal_right_hand(1) = -90 * DEG2RAD;//_x_right_hand(4);//-180 * DEG2RAD;
		// 		_rpy_goal_right_hand(2) = _x_right_hand(5);//90 * DEG2RAD;
		// 		reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		// }
		// else if (_cnt_plan == 3)
		// {	
		// 	cout << "_q_home(13) : " << _q_home(13)<<endl;
		// 	_q_home = _q;
		// 	_q_home(13) = 40 *DEG2RAD;
		// 	// _q(13) = 40 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 

		// 	reset_target(_time_plan(_cnt_plan), _q_home);
		// }

	}
}
void CController::control_mujoco()
{
	ModelUpdate();
	motionPlan();

	//Control
	if (_control_mode == 1) //joint space control
	{
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
			_bool_joint_motion = true;
		}
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();

		JointControl();

		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
	else if (_control_mode == 2 ) //task space hand control
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			LeftHandTrajectory.reset_initial(_start_time, _x_left_hand, _xdot_left_hand);
			LeftHandTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);

			RightHandTrajectory.reset_initial(_start_time, _x_right_hand, _xdot_right_hand);
			RightHandTrajectory.update_goal(_x_goal_right_hand, _xdot_goal_right_hand, _end_time);
			_bool_ee_motion = true;
		}
		LeftHandTrajectory.update_time(_t);

		_x_des_left_hand = LeftHandTrajectory.position_cubicSpline();
		_xdot_des_left_hand = LeftHandTrajectory.velocity_cubicSpline();

		RightHandTrajectory.update_time(_t);
		_x_des_right_hand = RightHandTrajectory.position_cubicSpline();
		_xdot_des_right_hand = RightHandTrajectory.velocity_cubicSpline();

		if (LeftHandTrajectory.check_trajectory_complete() == 1 || RightHandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
		OperationalSpaceControl();

	}
	
}
void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	//set Jacobian
	_J_hands.block<6, 14>(0, 0) = Model._J_left_hand;
	_J_hands.block<6, 14>(6, 0) = Model._J_right_hand;
	_J_T_hands = _J_hands.transpose();

	_J_ori_hands.block<3, 14>(0, 0) = Model._J_left_hand_ori;
	_J_ori_hands.block<3, 14>(3, 0) = Model._J_right_hand_ori;
	_J_ori_T_hands = _J_ori_hands.transpose();
	_J_pos_hands.block<3, 14>(0, 0) = Model._J_left_hand_pos;
	_J_pos_hands.block<3, 14>(3, 0) = Model._J_right_hand_pos;
	_J_pos_T_hands = _J_pos_hands.transpose();

	//calc Jacobian dot (with lowpass filter)
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 14; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}
	}
	_Jdot_qdot = _Jdot_hands * _qdot;


	// for (int i = 0; i < 12; i++)
	// {
	// 	_CT_force(i) = CustomMath::LowPassFilter(_dt, 2.0 * PI * 20.0, CT_Force(i), _pre_CT_force(i)); //low-pass filter

	// 	_pre_CT_force(i) = CT_Force(i);

	// }
	// CT_Torque = _J_T_hands * _CT_force;

	_x_left_hand.head(3) = Model._x_left_hand; // x, y, z
	_x_left_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_left_hand); // r, p, y
	_x_right_hand.head(3) = Model._x_right_hand; // x, y, z
	_x_right_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_right_hand); // r, p, y
	_xdot_left_hand = Model._xdot_left_hand;
	_xdot_right_hand = Model._xdot_right_hand;

	
}
void CController::JointControl()
{
	_torque.setZero();
	_kpj = 400.0;
	_kdj = 40.0;

	_torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot) ) + Model._bg;
	
	for (int i=0; i<7; i++) 
	{	
		log(i) = _q(i);
		log(i+7) = _q_des(i);
	}

	fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/test1.txt",ios::app);
	fout << log.transpose() <<endl;
	fout.close();

}
void CController::OperationalSpaceControl()
{	

	_torque.setZero();
	pre_torque.setZero();

	_kp = 400.0;
	_kd = 40.0;

	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);
	_x_err_right_hand = _x_des_right_hand.head(3) - Model._x_right_hand;
	_R_des_right_hand = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_xdot_err_left_hand = _xdot_des_left_hand.head(3) - Model._xdot_left_hand.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand.head(3) - Model._xdot_right_hand.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand.segment(3, 3); //only daming for orientation

	// 1st: hands pos and ori, 2nd: joint dampings
	_Lambda_hands.setZero();
	_J_bar_T_hands.setZero();
	_J_bar_T_hands = CustomMath::pseudoInverseQR(_J_T_hands);
	_Lambda_hands = _J_bar_T_hands * Model._A * CustomMath::pseudoInverseQR(_J_hands);
	_Null_hands = _Id_14 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();

	_xddot_star.segment(0, 3) = _kp * _x_err_left_hand + _kd * _xdot_err_left_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_left_hand + _kd * _Rdot_err_left_hand;//left hand orientation control
	_xddot_star.segment(6, 3) = _kp * _x_err_right_hand + _kd * _xdot_err_right_hand;//right hand position control
	_xddot_star.segment(9, 3) = _kp * _R_err_right_hand + _kd * _Rdot_err_right_hand;//right hand orientation control

	// Only body = motor(0) null control
	_q_des = _q;
	_q_des(0) = 0.0;
	_q_des(2) = 20.0 * DEG2RAD;
	_q_des(9) = -20.0 * DEG2RAD;

	// _torque = _J_T_hands * _Lambda_hands * _xddot_star + Model._bg;
	_torque = _J_T_hands * _Lambda_hands * _xddot_star +_Null_hands * Model._A * (_kp * (_q_des - _q)) + Model._bg ;

}


void CController::Initialize()
{	
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/lstm.txt");
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/test1.txt");
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/adadad.txt");
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/Oper1.txt");
	//////////////////////

	
	_control_mode = 2; //1: joint space, 2: operational space

	_bool_init = true;

	_t = 0.0;
	_init_t = 0.0;

	_pre_t = 0.0;
	_dt = 0.0;
	friction_coeff = 0;

	_q.setZero(_dofj);
	_qdot.setZero(_dofj);
	_torque.setZero(_dofj);


	_Fd.setZero(12);
	_Fd_error.setZero(12);

	_pre_q.setZero(_dofj);
	_pre_qdot.setZero(_dofj);

	_q_home.setZero(_dofj);
	
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/check_mob.txt");
    // ************ TEST ****************
	_q_home(0) = 0.0;
	_q_home(1) = 40.0 * DEG2RAD; //increased initial angle of LShP 		30.0 *  DEG2RAD; // LShP
	_q_home(8) = -40.0 * DEG2RAD; //increased initial angle of RShP 		30.0 *  DEG2RAD; // RShP
	_q_home(2) = 20.0 * DEG2RAD; // LShR
	_q_home(9) = -20.0 * DEG2RAD; // RShR
	_q_home(4) = 80.0 * DEG2RAD; //LElP
	_q_home(11) = -80.0 * DEG2RAD; //RElP
	_q_home(5) = -30.0 * DEG2RAD; //LWrP
	_q_home(12) = 30.0 * DEG2RAD; //RWrP
	// _q_home(6) = -30.0 * DEG2RAD; //LWrP
	// _q_home(13) = 30.0 * DEG2RAD; //RWrP
	_q_home(6) = 0.0 * DEG2RAD; //LWrP
	_q_home(13) = 0.0 * DEG2RAD; //RWrP

	// _q_home(0) = 0.0* DEG2RAD;
	// _q_home(1) = -20.0* DEG2RAD;
	// _q_home(2) = 35.0* DEG2RAD;
	_q_home(3) = 50.0* DEG2RAD;
	// _q_home(4) = 50.0* DEG2RAD;
	// _q_home(5) = 0.0* DEG2RAD;
	// _q_home(6) = 0.0* DEG2RAD;
	// _q_home(7) = 0.0* DEG2RAD;

	// _q_home(8) = +20.0* DEG2RAD;
	// _q_home(9) = -35.0* DEG2RAD;
	// _q_home(10) = -30.0* DEG2RAD;
	_q_home(11) = 50.0* DEG2RAD;
	// _q_home(12) = 0.0* DEG2RAD;
	// _q_home(13) = 0.0* DEG2RAD;//-0.000232359;
	// _q_home(14)= 0.0* DEG2RAD;// 0.00781449;

	// _M_ob.setZero(_dofj);
	// MOB param

	_torque_dist.setZero(_dofj);

	y_left = 0;
	y_right = 0;
	// ran_val.setZero(2);

	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_q_des.setZero(_dofj);
	_qdot_des.setZero(_dofj);

	_q_goal.setZero(_dofj);
	_qdot_goal.setZero(_dofj);

	_x_left_hand.setZero(6);
	_x_right_hand.setZero(6);
	_xdot_left_hand.setZero(6);
	_xdot_right_hand.setZero(6);

	_x_goal_left_hand.setZero(6);
	_xdot_goal_left_hand.setZero(6);
	_x_des_left_hand.setZero(6);
	_xdot_des_left_hand.setZero(6);
	_x_goal_right_hand.setZero(6);
	_xdot_goal_right_hand.setZero(6);
	_x_des_right_hand.setZero(6);
	_xdot_des_right_hand.setZero(6);

	_R_des_left_hand.setZero();
	_R_des_right_hand.setZero();

	error_f.setZero(12);
	we.setZero(12);
	x_zero.setZero(); //tem
	_R_left_current.setZero(); //tem
	_R_right_current.setZero();
	R_left_current.setZero();
	R_right_current.setZero();

	_xddot_star.setZero(12);
	_x_err_left_hand.setZero();
	_x_err_right_hand.setZero();
	_xdot_err_left_hand.setZero();
	_xdot_err_right_hand.setZero();
	_R_err_left_hand.setZero();
	_R_err_right_hand.setZero();
	_Rdot_err_left_hand.setZero();
	_Rdot_err_right_hand.setZero();

	_cnt_plan = 0;
	_bool_plan.setZero(2001);
	_time_plan.resize(2001);
	_time_plan.setConstant(5.0);
	_kpj = 400.0;
	_kdj = 40.0;
	_kp = 400.0;
	_kd = 40.0;

	Md.setZero(12,12);
	Md_I.setZero(12,12);
	Bd.setZero(12,12);
	Kd.setZero(12,12);

	//Sensor data
	// FT_left.setZero(6);
	// FT_right.setZero(6);
	// CT_Force.setZero(12);
	// _pre_CT_force.setZero(12);
	// _CT_force.setZero(12);
	// CT_Torque.setZero(15);

	_x_error.setZero(12);
	_xdot_error.setZero(12);

	//OSF

	_J_hands.setZero(12, 14);
	_Jdot_hands.setZero(12, 14);
	_Jdot_qdot.setZero(12);
	_pre_J_hands.setZero(12, 14);
	_pre_Jdot_hands.setZero(12, 14);
	_J_T_hands.setZero(14, 12);
	_Lambda_hands.setZero(12, 12);
	_Null_hands.setZero(14, 14);
	_J_bar_T_hands.setZero(12, 14);

	_J_pos_hands.setZero(6, 14);
	_J_pos_T_hands.setZero(14, 6);
	_J_ori_hands.setZero(6, 14);
	_J_ori_T_hands.setZero(14, 6);
	_Lambda_pos_hands.setZero(6, 6);
	_Lambda_ori_hands.setZero(6, 6);
	_Null_hands_pos.setZero(14, 14);
	_Null_hands_ori.setZero(14, 14);

	_pos_goal_left_hand.setZero();
	_rpy_goal_left_hand.setZero();
	_pos_goal_right_hand.setZero();
	_rpy_goal_right_hand.setZero();

	_Id_15.setIdentity(15, 15);
	_Id_14.setIdentity(14,14);
	_Id_12.setIdentity(12, 12);
	_Id_3.setIdentity(3,3);
	_Id_4.setIdentity(4,4);
	_Id_8.setIdentity(8,8);


	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_zero_vec.setZero(_dofj);

	JointTrajectory.set_size(14);
	LeftHandTrajectory.set_size(6);
	RightHandTrajectory.set_size(6);

	_bool_safemode = false;
	_dist_shoulder_hand_left = 0.0;
	_dist_shoulder_hand_right = 0.0;
	_workspace_avoid_gain = 0.0;
	_dir_hand_to_shoulder_left.setZero();
	_dir_hand_to_shoulder_right.setZero();
	_acc_workspace_avoid_left.setZero();
	_acc_workspace_avoid_right.setZero();

	obj_pose.setZero();

	log.setZero(14);
}


