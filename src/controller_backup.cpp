#include "controller.h"
#include <chrono>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
using namespace std;

CController::CController(int JDOF)
{
	_dofj = JDOF;
	Initialize();
}

CController::~CController()
{
}

void CController::read(double t, double* q, double* qdot, double* sensordata, double timestep, double* qacc)
{	
	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	
	for (int i = 0; i < 13; i++)
	{
		_q_co(i) = q[i];
		_qdot_co(i) = qdot[i]; //from simulator
		// cout << q[i]<<" ";
	}
	_q_co(13) = q[19];
	_qdot_co(13) = qdot[19];

	// for (int i = 0; i < 14; i++)
	// {
	// 	_q_co(i) = q[i];
	// 	_qdot_co(i) = qdot[i]; //from simulator
	// 	// cout << q[i]<<" ";
	// }
	// _q_co(14) = q[20];
	// _qdot_co(14) = qdot[20];
}

void CController::write(double* torque) // torque
{
	for (int i = 0; i < 15; i++)
	{
		torque[i] = _torque(i);
	} 
}
void CController::write_q(double* torque) // torque
{
	for (int i =0; i < 14; i++)
	{
		torque[i] = input_q(i);//_q_home_co(i);
	} 
	// for (int i =0; i < 14; i++)
	// {
	// 	torque[i] = 0;//_q_home_co(i);
	// } 

}
void CController::Wrist_IK(double p1, double p2)
{
	//Units 
	_pitch = p1;
	_yaw = p2 ;

	p_q(0) = 15.0*sqrt(2.0)*cos(_yaw)*sin(_pitch) - sqrt(8649.0 - pow(15.0*sqrt(2.0)*cos( _pitch) - 15.0*sqrt(2.0),2.0)
	- pow(15.0*sqrt(2.0)*cos(_yaw) - 15.0*sqrt(2.0) + 15.0*sqrt(2.0)*sin(_pitch)*sin(_yaw),2.0)) 
	- 15.0*sqrt(2.0)*sin(_yaw) + 154.0; //

	p_q(1) = 15*sqrt(2)*sin(_yaw) - sqrt(8649 - pow(15*sqrt(2)*cos(_pitch) - 15*sqrt(2),2) 
	- pow(15*sqrt(2) - 15*sqrt(2)*cos(_yaw) + 15*sqrt(2)*sin(_pitch)*sin(_yaw),2))
    + 15*sqrt(2)*cos(_yaw)*sin(_pitch) + 154;

	p_q(0) =  p_q(0)*(0.085/154)-0.033668831;
	p_q(1) =  p_q(1)*(0.085/154)-0.033668831;

	// cout <<"p_q: "<< p_q(0) <<" "<< p_q(1) << endl;
}
void CController::reset_target(double motion_time, VectorXd target_joint_position)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal_co = target_joint_position.head(14);
	// _q_goal_co = target_joint_position.head(15);
	_qdot_goal_co.setZero();
	// _qdot_goal = (_q_goal_co-_q_co)/_motion_time;
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
	_time_plan(1) = 5;
	_time_plan(2) = 5;//1.5 
	_time_plan(3) = 1.5;
	_time_plan(4) = 1.5; 
	_time_plan(5) = 1.5;
	_time_plan(6) = 1.5;
	_time_plan(7) = 1.5;
	_time_plan(8) = 1.5;
	_time_plan(9) = 1.5;
	_time_plan(10) = 1.5; 
	_time_plan(11) = 1.5;
	_time_plan(12) = 1.5;
	_time_plan(13) = 1.5;
	_time_plan(14) = 1.5;
	_time_plan(15) = 1.5;
	// _time_plan(9) = 3.0;
	// _time_plan(10) = 3.0;
	// _time_plan(11) = 3.0;
	// _time_plan(12) = 3.0;

	if (_bool_plan(_cnt_plan) == 1)
	{
		_cnt_plan = _cnt_plan + 1;
		if (_cnt_plan == 1)
		{	
			// p1 = 20*DEG2RAD;// pitch
			// p2 =0;// 20*DEG2RAD;// yaw
			// Wrist_IK(p1,p2);
			// _q_home_co_init(0) = 45*DEG2RAD;
			// _q_home_co_init(3) = 45*DEG2RAD;
			// _q_home_co_init(12) = p_q(0);
			// _q_home_co_init(13) = p_q(1);


			// _q_home_co_init(13) =  20.0 * DEG2RAD;
			// _q_home_co_init(14) =  0.0 * DEG2RAD;
			reset_target(_time_plan(_cnt_plan), _q_home_co);
		}
		
		if (_cnt_plan == 2)
		{	

			_pos_goal_left_hand(0) = 0.65;
            _pos_goal_left_hand(1) = _x_left_hand(1);
            _pos_goal_left_hand(2) = _x_left_hand(2);
			_rpy_goal_left_hand(0) = _x_left_hand(3);
            _rpy_goal_left_hand(1) = _x_left_hand(4);//90*DEG2RAD;
            _rpy_goal_left_hand(2) = _x_left_hand(5);

			_pos_goal_right_hand(0) = _x_right_hand(0);
            _pos_goal_right_hand(1) = _x_right_hand(1);
            _pos_goal_right_hand(2) = _x_right_hand(2);
			_rpy_goal_right_hand(0) = _x_right_hand(3);
            _rpy_goal_right_hand(1) = _x_right_hand(4);//60 * DEG2RAD;
            _rpy_goal_right_hand(2) = _x_right_hand(5);//0 * DEG2RAD;

        	reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
		}

		// if (_cnt_plan == 2)
		// {	
		// 	cout << "2" <<endl;
		// 	///////left//////
		// 	// _q_home_co(5) = 25.8*DEG2RAD;// pitch
		// 	// _q_home_co(6) = 16.4*DEG2RAD;// yaw
		// 	// _q_home_co_init(2) = 20*DEG2RAD;// yaw
		// 	// _q_home_co_init(11) = 40*DEG2RAD;// roll 
		// 	_q_home_co(5) = 15*DEG2RAD;

		// 	p1 = 0.0 * DEG2RAD;// pitch
		// 	p2 = 35.0 * DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 3)
		// {	
		// 	cout << "3" <<endl;
		// 	_q_home_co(11) = 30*DEG2RAD;
		// 	_q_home_co(4) = -30*DEG2RAD;
		// 	_q_home_co(5) = -5*DEG2RAD;
		// 	_q_home_co(6) = 0*DEG2RAD;
		// 	p1 = 0.0 * DEG2RAD;// pitch
		// 	p2 = -35.0 * DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 4)
		// {	
		// 	cout << "4" <<endl;
		// 	///////left//////
		// 	_q_home_co(4) = 30*DEG2RAD;
		// 	_q_home_co(5) = 15.0*DEG2RAD;// pitch
		// 	// _q_home_co(6) = 16.4*DEG2RAD;// yaw
		// 	// _q_home_co_init(2) = 20*DEG2RAD;// yaw
		// 	// _q_home_co_init(11) = 40*DEG2RAD;// roll 
		// 	_q_home_co(11) = -30*DEG2RAD;
		// 	p1 = 0.0 * DEG2RAD;// pitch
		// 	p2 = 30.0 * DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 5)
		// {	
		// 	cout << "5" <<endl;
		// 	_q_home_co(0) = -45 * DEG2RAD;
		// 	_q_home_co(7) = +45 * DEG2RAD;

		// 	_q_home_co(3) = 60 * DEG2RAD;
		// 	_q_home_co(10) = 60 * DEG2RAD;

		// 	_q_home_co(5) = 5.0*DEG2RAD;// pitch
		// 	_q_home_co(6) = -5.0 *DEG2RAD;// yaw
			

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 6)
		// {	
		// 	///////left//////
		// 	cout << "6" <<endl;
		// 	_q_home_co(1) = 60*DEG2RAD;// roll
		// 	_q_home_co(8) = -60*DEG2RAD;// roll

		// 	_q_home_co(2) = 50*DEG2RAD;// roll
		// 	_q_home_co(9) = -50*DEG2RAD;// roll
			
		// 	_q_home_co(5) = +25.0*DEG2RAD;// pitch
		// 	p1 = -15*DEG2RAD;// pitch
		// 	p2 =15;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 7)
		// {	
		// 	cout << "7" <<endl;
		// 	_q_home_co(1) = 20*DEG2RAD;// roll
		// 	_q_home_co(8) = -20*DEG2RAD;// roll

		// 	_q_home_co(5) = 0.0*DEG2RAD;// pitch
		// 	_q_home_co(6) = 0.0*DEG2RAD;// yaw


		// 	p1 = 0.0*DEG2RAD;// pitch
		// 	p2 = 0.0*DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);
		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 8)
		// {	
		// 	cout << "8" <<endl;
		// 	_q_home_co(0) = 45*DEG2RAD;// roll
		// 	_q_home_co(7) = -45*DEG2RAD;// roll

		// 	_q_home_co(1) = 10*DEG2RAD;// roll
		// 	_q_home_co(8) = -10*DEG2RAD;// roll

		// 	_q_home_co(2) = 0.0*DEG2RAD;// roll
		// 	_q_home_co(9) = 0.0*DEG2RAD;// roll

		// 	_q_home_co(3) = 100*DEG2RAD;// roll
		// 	_q_home_co(10) = 100*DEG2RAD;// roll

		// 	_q_home_co(4) = 0.0*DEG2RAD;// roll
		// 	_q_home_co(11) = 0.0*DEG2RAD;// roll

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }

		// if (_cnt_plan == 9)
		// {	
		// 	cout << "9" <<endl;
		// 	///////left//////
	
		// 	_q_home_co(5) = 25*DEG2RAD;

		// 	p1 = 0.0 * DEG2RAD;// pitch
		// 	p2 = 35.0 * DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 10)
		// {	
		// 	cout << "10" <<endl;
			
		// 	_q_home_co(5) = -25*DEG2RAD;
		// 	_q_home_co(6) = 0*DEG2RAD;
		// 	p1 = 0.0 * DEG2RAD;// pitch
		// 	p2 = -35.0 * DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 11)
		// {	
		// 	cout << "11" <<endl;
		// 	///////left//////
		// 	_q_home_co(5) = 0.0*DEG2RAD;
		// 	_q_home_co(6) = -20*DEG2RAD;

		// 	p1 = -35.0 * DEG2RAD;// pitch
		// 	p2 = 0.0 * DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 12)
		// {	
		// 	cout << "12" <<endl;
		// 	///////left//////
		// 	_q_home_co(5) = 0.0*DEG2RAD;
		// 	_q_home_co(6) = 20*DEG2RAD;

		// 	p1 = 35.0 * DEG2RAD;// pitch
		// 	p2 = 0.0 * DEG2RAD;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

			

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 13)
		// {	
		// 	///////left//////
			
		// 	_q_home_co(4) = 90.0*DEG2RAD;// pitch
		// 	_q_home_co(5) = -20.0*DEG2RAD;// pitch

		// 	_q_home_co(11) = -90.0*DEG2RAD;// pitch
		// 	p1 = -35.0 *DEG2RAD;// pitch
		// 	p2 =0.0;// 20*DEG2RAD;// yaw
		// 	Wrist_IK(p1,p2);
		// 	_q_home_co(12) = p_q(0);
		// 	_q_home_co(13) = p_q(1);

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 14)
		// {	

		// 	_q_home_co(4) = -90.0*DEG2RAD;// pitc

		// 	_q_home_co(11) = 90.0*DEG2RAD;// pitch

		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }
		// if (_cnt_plan == 15)
		// {	
		// 	_q_home_co.setZero();
		// 	reset_target(_time_plan(_cnt_plan), _q_home_co);
		// }

	}
}

void CController::control_mujoco()
{
	ModelUpdate();
	motionPlan();
	if (_control_mode == 1) //joint space control
	{
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{	
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;

			JointTrajectory.reset_initial(_start_time, _q_des_co, _qdot_des_co);
			JointTrajectory.update_goal(_q_goal_co, _qdot_goal_co, _end_time);
			_bool_joint_motion = true;

		}
		JointTrajectory.update_time(_t);
		_q_des_co = JointTrajectory.position_cubicSpline();
		_qdot_des_co = JointTrajectory.velocity_cubicSpline();
		// JointControl();
		TEST_control();
		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
	else if (_control_mode == 2) //task space hand control
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;

			LeftHandTrajectory.reset_initial(_start_time, _x_des_left_hand, _xdot_left_hand_co);
			// LeftHandTrajectory.reset_initial(_start_time, _x_left_hand_co, _xdot_left_hand_co);
			LeftHandTrajectory.update_goal(_x_goal_left_hand, _xdot_goal_left_hand, _end_time);

			RightHandTrajectory.reset_initial(_start_time, _x_right_hand_co, _xdot_right_hand_co);
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
		if (_control_mode == 2)
		{
			CLIK();
		}
	}
	
}
void CController::ModelUpdate()
{
	Model.update_kinematics(_q_co, _qdot_co);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();


	_J_hands_co.block<6, 14>(0, 0) = Model._J_left_hand_co;
	_J_hands_co.block<6, 14>(6, 0) = Model._J_right_hand_co;
	_J_T_hands_co = _J_hands_co.transpose();

	_J_ori_hands_co.block<3, 14>(0, 0) = Model._J_left_hand_ori_co;
	_J_ori_hands_co.block<3, 14>(3, 0) = Model._J_right_hand_ori_co;
	_J_ori_T_hands_co = _J_ori_hands_co.transpose();
	_J_pos_hands_co.block<3, 14>(0, 0) = Model._J_left_hand_pos_co;
	_J_pos_hands_co.block<3, 14>(3, 0) = Model._J_right_hand_pos_co;
	_J_pos_T_hands_co = _J_pos_hands_co.transpose();

	_x_left_hand_co.head(3) = Model._x_left_hand_co; // x, y, z
	_x_left_hand_co.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_left_hand_co); // r, p, y
	_x_right_hand_co.head(3) = Model._x_right_hand_co; // x, y, z
	_x_right_hand_co.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_right_hand_co); // r, p, y
	_xdot_left_hand_co = Model._xdot_left_hand_co;
	_xdot_right_hand_co = Model._xdot_right_hand_co;
}
void CController::TEST_control()
{	
	pos_error = pos_error + (_q_des_co - _q_co)*_dt;
	// input_q = 500 * (_q_des_co - _q_co) +1 *pos_error ; //velocity mode
	// input_q = 120 * (_q_des_co - _q_co) + pos_error;
	input_q = _q_des_co;// + pos_error;
	
}
void CController::CLIK()
{
	// _torque.setZero();
	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand_co;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand_co, _R_des_left_hand);
	_x_err_right_hand = _x_des_right_hand.head(3) - Model._x_right_hand_co;
	_R_des_right_hand = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand_co, _R_des_right_hand);

	_x_des_err_co.segment(0, 3) = _x_err_left_hand;
	_x_des_err_co.segment(3, 3) = _R_err_left_hand;
	_x_des_err_co.segment(6, 3) = _x_err_right_hand;
	_x_des_err_co.segment(9, 3) = _R_err_right_hand;

	_xdot_err_left_hand = _xdot_des_left_hand.head(3) - Model._xdot_left_hand_co.segment(0, 3);
	_Rdot_err_left_hand = -Model._xdot_left_hand_co.segment(3, 3); //only daming for orientation
	_xdot_err_right_hand = _xdot_des_right_hand.head(3) - Model._xdot_right_hand_co.segment(0, 3);
	_Rdot_err_right_hand = -Model._xdot_right_hand_co.segment(3, 3); //only daming for orientation


	VectorXd x_des_L(6);
    VectorXd x_des_R(6);
	x_des_L << -0.6, 0.253, 1.0, 1.065, -0.091+sin(_t)/2, 1.578+cos(_t)/2;
    x_des_R << 0.799, -0.253, 1.2, 1.065, -0.091+sin(_t)/2, 1.578+cos(_t)/2;

	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand_co;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand_co, _R_des_left_hand);


	// _xdot_des_co.segment(0, 6) = _xdot_des_left_hand;
	// _xdot_des_co.segment(6, 6) = _xdot_des_right_hand;
	_xdot_des_co.segment(0, 3) = _xdot_err_left_hand;
	_xdot_des_co.segment(3, 3) = _Rdot_err_left_hand;
	_xdot_des_co.segment(6, 3) = _xdot_err_right_hand;
	_xdot_des_co.segment(9, 3) = _Rdot_err_right_hand;

	x_curr_l_co.segment(0, 3) = _x_err_left_hand;
	x_curr_l_co.segment(3, 3) = _R_err_left_hand;

	// _qdot_des_co =  CustomMath::DampedWeightedPseudoInverse(Model._J_left_hand_co, 1*_Id_14, true) * (_xdot_des_left_hand + 10 *(x_curr_l_co));
	// _qdot_des_co = CustomMath::DampedWeightedPseudoInverse(_J_hands_co, 0.001*_Id_14, true) * (2*_xdot_des_co + 2 *(_x_des_err_co));
	// _qdot_des_co = CustomMath::pseudoInverseQR(Model._J_left_hand_co) * (_xdot_des_co + 10 *(x_curr_l_co));
	qdotd =  CustomMath::pseudoInverseQR(_J_hands_co) * (_xdot_des_co + 10 *(_x_des_err_co));
	qdotd.setZero();
	qdotd(0) = 0.01;
	qd = qd + qdotd * _dt;

	input_q = qd;
	// p1 = qd(12);
	// p2 = qd(13);
	// Wrist_IK(p1, p2);
	// input_q(12) = p_q(0);
	// input_q(13) = p_q(1);

}
void CController::JointControl()
{
	// _torque.setZero();
	_kpj = 400.0;//400.0
	_kdj = 40.0;

	// _q_des_co = _q_home_co;//
	// _torque_co = Model._A_co*(_kpj*(_q_des_co - _q_co) + _kdj*(_qdot_des_co - _qdot_co)) + Model._bg_co;
}
void CController::Initialize()
{	
	//////////////////////

	_J_bar_T_hands_co.setZero(12,14);

	p_q.setZero(2);
	_control_mode = 1; //1: joint space, 2: operational space
	_bool_init = true;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_torque.setZero(_dofj);
	_torque_co.setZero(14);

	_q_home_co.setZero(14);
	_q_home_co_init.setZero(14);

	// _torque_co.setZero(15);

	// _q_home_co.setZero(15);
	// _q_home_co_init.setZero(15);

	// _q_home.setZero();
	// _q_home_co(0) = 40.0 * DEG2RAD; //increased initial angle of LShP 		30.0 *  DEG2RAD; // LShP
	// _q_home_co(7) = -40.0 * DEG2RAD; //increased initial angle of RShP 		30.0 *  DEG2RAD; // RShP
	// _q_home_co(1) = 20.0 * DEG2RAD; // LShR
	// _q_home_co(8) = -20.0 * DEG2RAD; // RShR
	// _q_home_co(3) = 80.0 * DEG2RAD; //LElP
	// _q_home_co(10) = -80.0 * DEG2RAD; //RElP
	// // _q_home_co(5) = -30.0 * DEG2RAD; //LWrP
	// // _q_home_co(12) = 30.0 * DEG2RAD; //RWrP

	_q_home_co(0) = -20.0* DEG2RAD;
	_q_home_co(1) = 35.0* DEG2RAD;
	_q_home_co(2) = 30.0* DEG2RAD;
	_q_home_co(3) = 50.0* DEG2RAD;
	_q_home_co(4) = 0.0* DEG2RAD;
	_q_home_co(5) = 0.0* DEG2RAD;
	_q_home_co(6) = 0.0* DEG2RAD;

	_q_home_co(7) = +20.0* DEG2RAD;
	_q_home_co(8) = -35.0* DEG2RAD;
	_q_home_co(9) = -30.0* DEG2RAD;
	_q_home_co(10) = 50.0* DEG2RAD;
	_q_home_co(11) = 0.0* DEG2RAD;
	_q_home_co(12) = 0.0* DEG2RAD;//-0.000232359;
	_q_home_co(13) = 0.0* DEG2RAD;// 0.00781449;
	
	std::remove("../data/Sim_data/t4.txt");
	std::remove("../data/Sim_data/stack_CLIK.txt");
	stack.setZero(28);
	stack_CLIK.setZero(28);
	model_hands.setZero(12);

	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	pos_error.setZero(14);

	input_q.setZero(14);
	_q_des_co.setZero(14);
	_qdot_des_co.setZero(14);
	_pre_qdot_des_co.setZero(14);

	_q_goal.setZero(14);
	_qdot_goal.setZero(14);

	// pos_error.setZero(15);
	// input_q.setZero(15);
	// _q_des_co.setZero(15);
	// _qdot_des_co.setZero(15);
	// _pre_qdot_des_co.setZero(15);
	// _q_goal.setZero(15);
	// _qdot_goal.setZero(15);

	// x_goal_co.setZero(12);
	// x_curr_co.setZero(12);
	// xdot_curr_co.setZero(12);

	x_goal_l_co.setZero(6);
	x_goal_r_co.setZero(6);
	x_curr_l_co.setZero(6);
	x_curr_r_co.setZero(6);
	pre_x_des_R.setZero(6);
	pre_x_des_L.setZero(6);
	

	_x_left_hand.setZero(6);
	_x_right_hand.setZero(6);
	_xdot_left_hand.setZero(6);
	_xdot_right_hand.setZero(6);

	_x_left_hand_co.setZero(6);
	_x_right_hand_co.setZero(6);
	_xdot_left_hand_co.setZero(6);
	_xdot_right_hand_co.setZero(6);

	_x_des_err_co.setZero(12);
	_xdot_des_co.setZero(12);

	_q_co.setZero(14);
	_qdot_co.setZero(14);
	_q_goal_co.setZero(14);
	_qdot_goal_co.setZero(14);
	_dig_A_co.setZero(14,14);

	// _q_co.setZero(15);
	// _qdot_co.setZero(15);
	// _q_goal_co.setZero(15);
	// _qdot_goal_co.setZero(15);
	// _dig_A_co.setZero(15,15);


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
	_bool_plan.setZero(701);
	_time_plan.resize(701);
	_time_plan.setConstant(5.0);
	_kpj = 400.0;
	_kdj = 40.0;
	_kp = 400.0;
	_kd = 40.0;

	
	//OSF
	_J_hands_co.setZero(12, 14);
	_J_T_hands_co.setZero(14, 12);
	_J_pos_hands_co.setZero(6, 14);
	_J_pos_T_hands_co.setZero(14, 6);
	_J_ori_hands_co.setZero(6, 14);
	_J_ori_T_hands_co.setZero(14, 6);

	// _J_hands_co.setZero(12, 15);
	// _J_T_hands_co.setZero(15, 12);
	// _J_pos_hands_co.setZero(6, 15);
	// _J_pos_T_hands_co.setZero(15, 6);
	// _J_ori_hands_co.setZero(6, 15);
	// _J_ori_T_hands_co.setZero(15, 6);

	_Id_15.setIdentity(15, 15);
	_Id_14.setIdentity(14,14);
	_Id_12.setIdentity(12, 12);
	_Id_3.setIdentity(3,3);
	_Id_4.setIdentity(4,4);

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_zero_vec.setZero(_dofj);

	JointTrajectory.set_size(14);
	// JointTrajectory.set_size(15);

	LeftHandTrajectory.set_size(6);
	RightHandTrajectory.set_size(6);

	_pos_goal_left_hand.setZero();
	_rpy_goal_left_hand.setZero();
	_pos_goal_right_hand.setZero();
	_rpy_goal_right_hand.setZero();

	_Lambda_hands.setZero(12,12);
	pre_qddddd.setZero(14);
	pre_qddddd_dot.setZero(14);
	qd.setZero(14);
	qdotd.setZero(14);



}


