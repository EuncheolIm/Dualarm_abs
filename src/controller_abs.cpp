#include "controller_abs.h"
#include <chrono>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace std;

CController::CController(int JDOF)
{
	_dofj = JDOF;
	Initialize();
}

CController::~CController()
{
}

void CController::read(double t, double* q, double* qdot)
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
		_qdot(i) = qdot[i]; 
	}

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

	_q_goal = target_joint_position.head(_dofj);
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
void CController::reset_targetAbs(double motion_time, Eigen::Matrix<double, 6, 1> target_pose_abs, Eigen::Matrix<double, 6, 1> target_pose_rel){
	_control_mode = 3;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_abs = target_pose_abs;
	_xdot_goal_abs.setZero();

	_x_goal_rel = target_pose_rel;
	_xdot_goal_rel.setZero();
}

void CController::motionPlan()
{
	_time_plan(1) = 5;
	_time_plan(2) = 5;//1.5 


	if (_bool_plan(_cnt_plan) == 1)
	{
		_cnt_plan = _cnt_plan + 1;
		if (_cnt_plan == 1)
		{	
			reset_target(_time_plan(_cnt_plan), _q_home);
		}
		
		// if (_cnt_plan == 2)
		// {	
		// 	_abs_goal(0) += 0.1;
        //     _abs_goal = p_a_pose;
		// 	_rel_goal = p_r_pose;
		// 	_rel_goal(0) = 0.0;
		// 	_rel_goal(1) = -0.3;
		// 	_rel_goal(2) = 0.0;
        // 	reset_targetAbs(_time_plan(_cnt_plan), _abs_goal, _rel_goal);
		// }
		// if (_cnt_plan == 3)
		// {	

        //     _abs_goal(4) -= 30*DEG2RAD;
		// 	_rel_goal = p_r_pose;
		// 	_rel_goal(0) = 0.0;
		// 	_rel_goal(1) = -0.3;
		// 	_rel_goal(2) = 0.0;
        // 	reset_targetAbs(_time_plan(_cnt_plan), _abs_goal, _rel_goal);
		// }
		// if (_cnt_plan == 4)
		// {	

        //     _abs_goal(4) += 60*DEG2RAD;
		// 	_rel_goal = p_r_pose;
		// 	_rel_goal(0) = 0.0;
		// 	_rel_goal(1) = -0.3;
		// 	_rel_goal(2) = 0.0;
        // 	reset_targetAbs(_time_plan(_cnt_plan), _abs_goal, _rel_goal);
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
	else if (_control_mode == 2) //task space hand control
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;

			LeftHandTrajectory.reset_initial(_start_time, _x_des_left_hand, _xdot_left_hand);

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
		// if (_control_mode == 2)
		// {
			// CLIK();
		// }
	}
	else if (_control_mode == 3) //task space hand control
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;

			absTrajectory.reset_initial(_start_time, p_a_pose, pdot_a_pose);
			absTrajectory.update_goal(_x_goal_abs, _xdot_goal_abs, _end_time);

			relTrajectory.reset_initial(_start_time, p_r_pose, pdot_r_pose);
			relTrajectory.update_goal(_x_goal_rel, _xdot_goal_rel, _end_time);
			_bool_ee_motion = true;
		}
		absTrajectory.update_time(_t);
		p_a_des.head(3) = absTrajectory.position_cubicSpline();
		R_a_des = absTrajectory.rotationCubic();
		p_a_des.segment<3>(3) = CustomMath::GetBodyRotationAngle(R_a_des);
		pdot_a_des.head(3) = absTrajectory.velocity_cubicSpline();
		pdot_a_des.segment<3>(3) = absTrajectory.rotationCubicDot();

		relTrajectory.update_time(_t);
		p_r_des.head(3) = relTrajectory.position_cubicSpline();
		R_r_des = relTrajectory.rotationCubic();
		p_r_des.segment<3>(3) = CustomMath::GetBodyRotationAngle(R_r_des);
		pdot_r_des.head(3) = relTrajectory.velocity_cubicSpline();
		pdot_r_des.segment<3>(3) = relTrajectory.rotationCubicDot();

		AbsIK();
		if (absTrajectory.check_trajectory_complete() == 1 || relTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}

	}
	
}
void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	_J_l_hand = Model._J_left_hand;
	_J_r_hand = Model._J_right_hand.block(0,7,6,7);
	// 1 : Left, 2 : Right
	p_a = (Model._x_left_hand + Model._x_right_hand)/2;
	p_r = Model._x_right_hand - Model._x_left_hand;
	////////////////////////////////////////////////////////////////
	///////  get R_a
	Eigen::Quaterniond q1(Model._R_left_hand);
    Eigen::Quaterniond q2(Model._R_right_hand);
    Eigen::Quaterniond q_mid = q1.slerp(0.5, q2); // SLERP 보간 (t=0.5: 중간 회전)
    Matrix3d R_mid = q_mid.toRotationMatrix();

	R_a = Model._R_left_hand * R_mid;
	///////  get R_a
	////////////////////////////////////////////////////////////////
	R_r = Model._R_left_hand.transpose() * Model._R_right_hand;

	p_a_pose.head(3) = p_a;
	p_a_pose.tail(3) = CustomMath::GetBodyRotationAngle(R_a);

	p_r_pose.head(3) = p_r;
	p_r_pose.tail(3) = CustomMath::GetBodyRotationAngle(R_r);

	// cout << "Model._x_left_hand: "<<Model._x_left_hand.transpose() << endl;
	// cout << "Model._x_right_hand: "<<Model._x_right_hand.transpose() << endl;
	// cout << "p_a: "<<p_a.transpose() << endl;
	// cout << "p_r: "<<p_r.transpose() << endl<< endl;

	pdot_a =(Model._xdot_right_hand.head(3) + Model._xdot_left_hand.head(3))/2;
	pdot_r = Model._xdot_right_hand.head(3) - Model._xdot_left_hand.head(3);
	// omega = xdot_.. tail(3);
	omega_a = (Model._xdot_right_hand.tail(3) + Model._xdot_left_hand.tail(3))/2;
	omega_r = Model._xdot_right_hand.tail(3) - Model._xdot_left_hand.tail(3);

	pdot_a_pose.head(3) = pdot_a;
	pdot_a_pose.tail(3) = omega_a;

	pdot_r_pose.head(3) = pdot_r;
	pdot_r_pose.tail(3) = omega_r;

	Rdot_a = CustomMath::skew(omega_a)*R_a;
}

void CController::AbsIK(){
	// absolute Jacobian
	J_a.block(0,0,6,EACH_DOF) = 0.5*_J_l_hand;
	J_a.block(0,EACH_DOF,6,EACH_DOF) = 0.5*_J_r_hand;
	// relative Jacobian
	J_r.block(0,0,6,EACH_DOF) = -_J_l_hand;
	J_r.block(0,EACH_DOF,6,EACH_DOF) = _J_r_hand;

	J_ar.block(0,0,6,2*EACH_DOF) = J_a;
	J_ar.block(6,0,6,2*EACH_DOF) = J_r;

	e_a.segment(0,3) = p_a_des.head(3) - p_a;
	e_a.segment(3,3) = -CustomMath::getPhi(R_a, R_a_des); // R_a_des

	Vector3d a_p_rd, a_pdot_rd;
	a_p_rd.setZero();
	a_pdot_rd.setZero();

	// p_r_des<< 0,0,-0.3;
	// a_p_rd = p_r_des.head(3) - p_a.head(3) ;
	a_p_rd = R_a.transpose()* p_r_des.head(3) ;
	// a_pdot_rd = pdot_r_des.head(3) - pdot_a.head(3);
	a_pdot_rd = R_a.transpose()* (pdot_r_des.head(3) - CustomMath::skew(omega_a)* p_r_des.head(3));

	R_r_des.setIdentity();
	e_r.segment(0,3) = R_a * a_p_rd - p_r;
	e_r.segment(3,3) = -Model._R_left_hand * CustomMath::getPhi(R_r, R_r_des); // R_r_des

	e_ar.segment(0,6) = e_a;
	e_ar.segment(6,6) = e_r;

	v_a = pdot_a_des; // pdot_a_des.head(3)
	// v_r.segment(0,3) = R_a* a_pdot_rd + Rdot_a* a_p_rd; // pdot_a_r_des.head(3), p_a_r_des.head(3)
	v_r.segment(0,3) = R_a* a_pdot_rd + 
	(CustomMath::skew(Model._xdot_left_hand.tail(3)) + 0.5*CustomMath::skew(Model._xdot_right_hand.tail(3) - Model._xdot_left_hand.tail(3)))*R_a*a_p_rd; // pdot_a_r_des.head(3), p_a_r_des.head(3)
	v_r.segment(3,3) = pdot_r_des; // pdot_r_des.tail(3)


	v_des.segment(0,6) = v_a;
	v_des.segment(6,6) = v_r;

	_qdot_ar = CustomMath::pseudoInverseQR(J_ar) * (v_des + 10 * e_ar);
	_q_ar += _qdot_ar *_dt;

	_torque = Model._A * (400 * (_q_ar - _q) + 40* (_qdot_ar - _qdot)) + Model._bg;

	// cout << "abs_goal: \n" << _abs_goal.transpose()<<endl;
	// cout << "abs_pose: \n" << p_a_pose.transpose()<<endl;
	// cout << "rel_goal: \n" << _rel_goal.transpose()<<endl;
	// cout << "rel_pose: \n" << p_r_pose.transpose()<<endl<<endl;

}

void CController::JointControl()
{
	_torque.setZero();
	_kpj = 400.0;//400.0
	_kdj = 40.0;

	_torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot)) + Model._bg;

	_q_ar = _q;

	// std::cout << "_q_goal: "<<_q_goal.transpose() << std::endl;
	// std::cout << "_q_des: "<<_q_des.transpose() << std::endl;
	// std::cout << "_q: "<<_q.transpose() << std::endl << std::endl;
}
void CController::Initialize()
{	
	//////////////////////
	
	_control_mode = 1; //1: joint space, 2: operational space
	_bool_init = true;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_torque.setZero(_dofj);

	_q_home.setZero(_dofj);
	_q_home(1) = 10 * DEG2RAD;//-q_rand_val[10];//45 * DEG2RAD;
	_q_home(2) = 10 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 
	_q_home(4) = 50 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 

	_q_home(8) = -10 * DEG2RAD;//-q_rand_val[10];//45 * DEG2RAD;
	_q_home(9) = -10 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 
	_q_home(11) = -50 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD; 
	

	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;


	_q.setZero(_dofj);
	_qdot.setZero(_dofj);

	
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
	_bool_plan.setZero(701);
	_time_plan.resize(701);
	_time_plan.setConstant(5.0);
	_kpj = 400.0;
	_kdj = 40.0;
	_kp = 400.0;
	_kd = 40.0;

	_J_l_hand.setZero();
	_J_r_hand.setZero();

	p_r.setZero();
	p_a.setZero();
	R_r.setZero();
	R_a.setZero();
	Rdot_a.setZero();

	pdot_r.setZero();
	pdot_a.setZero();
	omega_r.setZero();
	omega_a.setZero();

	J_a.setZero();
	J_r.setZero();
	J_ar.setZero();

	e_a.setZero();
	e_r.setZero();

	v_a.setZero();
	v_r.setZero();
	v_des.setZero();

	p_a_des.setZero();
	p_r_des.setZero();
	pdot_a_des.setZero();
	pdot_r_des.setZero();
	R_a_des.setZero();
	R_r_des.setZero();

	p_r_pose.setZero();
	p_a_pose.setZero();
	pdot_r_pose.setZero();
	pdot_a_pose.setZero();

	_x_goal_abs.setZero();
	_x_goal_rel.setZero();
	_xdot_goal_abs.setZero();
	_xdot_goal_rel.setZero();

	_qdot_ar.setZero();
	_q_ar.setZero();

	_abs_goal.setZero();
	_rel_goal.setZero();


	_Id_15.setIdentity(15, 15);
	_Id_14.setIdentity(14,14);
	_Id_12.setIdentity(12, 12);
	_Id_3.setIdentity(3,3);
	_Id_4.setIdentity(4,4);

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_zero_vec.setZero(_dofj);

	JointTrajectory.set_size(_dofj);
	// JointTrajectory.set_size(15);

	LeftHandTrajectory.set_size(6);
	RightHandTrajectory.set_size(6);

	_pos_goal_left_hand.setZero();
	_rpy_goal_left_hand.setZero();
	_pos_goal_right_hand.setZero();
	_rpy_goal_right_hand.setZero();

	_Lambda_hands.setZero(12,12);



}

