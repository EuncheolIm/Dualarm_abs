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
	Initialize_MPC();
	
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

    //  contact force
	CT_Force.block<6, 1>(0, 0) = FT_left;
	CT_Force.block<6, 1>(6, 0) = FT_right;

	for(int i=0; i<7; i++)
	{
		obj_pose(i) = object_pose[i];
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

	_q_goal = target_joint_position.head(15);
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
	_time_plan(1) = 4.0; //move home position
	_time_plan(2) = 5.0; //move home position
	_time_plan(3) = 5.0; //move home position
	_time_plan(4) = 5.0; //move home position
	_time_plan(5) = 5.0; //move home position
	_time_plan(6) = 5.0; //move home position
	// srand(time(NULL));
	// for (int i = 2; i < 2000; i++)
	// {	
	// 	_time_plan(i) = (rand() % 76)*0.1 + 0.5; // 5 ~ 7 sec 0 ~7.5 +0.5 -> 8.0 
	// 	// _time_plan(i) = (rand() % 51)*0.1 ; 
	// }
	// _time_plan(4) = 100.0; //move home position
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

		else if (_cnt_plan == 2)
		{

				_pos_goal_left_hand(0) = _x_left_hand(0) +0.2;
				_pos_goal_left_hand(1) = _x_left_hand(1)-0.025;
				_pos_goal_left_hand(2) = _x_left_hand(2)+0.25;
				_rpy_goal_left_hand(0) = _x_left_hand(3);
				_rpy_goal_left_hand(1) = -90 * DEG2RAD;//_x_left_hand(4);
				_rpy_goal_left_hand(2) = _x_left_hand(5);

				_pos_goal_right_hand(0) = _x_right_hand(0) + 0.2;
				_pos_goal_right_hand(1) = _x_right_hand(1)+0.025;
				_pos_goal_right_hand(2) = _x_right_hand(2)+0.25;
				_rpy_goal_right_hand(0) = _x_right_hand(3);//0;//080 * DEG2RAD;
				_rpy_goal_right_hand(1) = -90 * DEG2RAD;//_x_right_hand(4);//-180 * DEG2RAD;
				_rpy_goal_right_hand(2) = _x_right_hand(5);//90 * DEG2RAD;

				// cout <<_pos_goal_left_hand.transpose() << endl;
				// cout <<_pos_goal_right_hand.transpose() << endl;

				// _pos_goal_right_hand(0) = 0.5;
				// _pos_goal_right_hand(1) = -0.25;
				// _pos_goal_right_hand(2) = 0.65;
				// _rpy_goal_right_hand(0) = 0;//080 * DEG2RAD;
				// _rpy_goal_right_hand(1) = -180 * DEG2RAD;
				// _rpy_goal_right_hand(2) = 90 * DEG2RAD;

				reset_target(_time_plan(_cnt_plan), _pos_goal_left_hand, _rpy_goal_left_hand, _pos_goal_right_hand, _rpy_goal_right_hand);
				// cout << "t: " << _time_plan(_cnt_plan)<<  "  ,x: "<< _pos_goal_left_hand.transpose() << "  ,q: "<< (_pos_goal_joint*RAD2DEG).transpose()<< endl;
		}
	

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

		// if (freq_check == loop)
		// {	
		// 	b_100hz_t = true;
		// 	if (b_100hz_t == true)
		// 	{
		// 		_100hz_t = _t;
		// 		_100hz_t = _100hz_t - 0.01;	
	
		// 		b_100hz_t = false;
		// 	}
		// 	for(int i=0; i<Np; i++)
		// 	{	
		// 		_100hz_t = _100hz_t + 0.01;
		// 		JointTrajectory.update_time(_t);
		// 		_q_des = JointTrajectory.position_cubicSpline();
		// 		_qdot_des = JointTrajectory.velocity_cubicSpline();

		// 		Y_temp.segment(0,14) = _q_des.segment(1,14); 		// _q_des
		// 		Y_temp.segment(14,14) = _qdot_des.segment(1,14); 	// _qdot_des
		// 		Y_des.segment(28*i,28) = Y_temp;
		// 	}
		// }
		// MPC();

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
		// OperationalSpaceControl();
		CLIK();


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
	_J_hands.block<6, 15>(0, 0) = Model._J_left_hand;
	_J_hands.block<6, 15>(6, 0) = Model._J_right_hand;
	_J_T_hands = _J_hands.transpose();

	_J_ori_hands.block<3, 15>(0, 0) = Model._J_left_hand_ori;
	_J_ori_hands.block<3, 15>(3, 0) = Model._J_right_hand_ori;
	_J_ori_T_hands = _J_ori_hands.transpose();
	_J_pos_hands.block<3, 15>(0, 0) = Model._J_left_hand_pos;
	_J_pos_hands.block<3, 15>(3, 0) = Model._J_right_hand_pos;
	_J_pos_T_hands = _J_pos_hands.transpose();

	//calc Jacobian dot (with lowpass filter)
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 15; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}
	}
	_Jdot_qdot = _Jdot_hands * _qdot;


	for (int i = 0; i < 12; i++)
	{
		_CT_force(i) = CustomMath::LowPassFilter(_dt, 2.0 * PI * 20.0, CT_Force(i), _pre_CT_force(i)); //low-pass filter

		_pre_CT_force(i) = CT_Force(i);

	}
	CT_Torque = _J_T_hands * _CT_force;

	_x_left_hand.head(3) = Model._x_left_hand; // x, y, z
	_x_left_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_left_hand); // r, p, y
	_x_right_hand.head(3) = Model._x_right_hand; // x, y, z
	_x_right_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_right_hand); // r, p, y
	_xdot_left_hand = Model._xdot_left_hand;
	_xdot_right_hand = Model._xdot_right_hand;

	_diag_A = (Model._A.diagonal()).asDiagonal();

}
void CController::SetQPProblem(int num_vars, int num_cons) {
    qp.InitializeProblemSize(num_vars, num_cons);
    _H.setZero(num_vars, num_vars);
    _g.setZero(num_vars);
    _ubA.setZero(num_cons);
    _lbA.setZero(num_cons);
}

// void CController::EstimateForce()
// {
// 	SetQPProblem(12,);

// 	_Pddot_des = 100*(x_targets_d.head(3) - Model._x_com) + 10*(x_targets_dot_d.head(3) - Model._com_vel);
// 	// _R_err = R_x_targets_d*Model._R_base.transpose();

// 	_wdot_des = kp_w_ct*(AngleAxisd(_R_err).axis()* AngleAxisd(_R_err).angle()) + kd_w_ct*(ctrl_->x_targets_dot_d.tail(3) - ctrl_->Model._R_base*ctrl_->_Rdot_base);
    
//     _b.head(3) = ctrl_->Tot_mass *(_Pddot_des + _gravity);
// 	_b.tail(3) = ctrl_->Model._A_virtual.block(3,3,3,3) * _wdot_des;

// 	Eigen::Matrix<double, 6, 12> A_wc;
// 	A_wc.block(0,0,3,3) = -_Id_3;	A_wc.block(0,6,3,3) = -_Id_3;
// 	A_wc.block(3,0,3,3) = CustomMath::skew(Model._x_left_hand - Model._x_base); A_wc.block(3,3,3,3) = -_Id_3; 
// 	A_wc.block(3,6,3,3) = CustomMath::skew(Model._x_right_hand - Model._x_base); A_wc.block(3,9,3,3) = -_Id_3; 
// }


void CController::JointControl()
{
	_torque.setZero();
	_kpj = 400.0;
	_kdj = 40.0;

	// _torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot) ) + Model._bg;
	_torque = _diag_A *(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot) ) + Model._bg;

	_q_des_clik = _q;

	cout << "_q_goal: "<< _q_goal.transpose() << endl;
	cout << "_q_des: "<< _q_des.transpose() << endl;
	cout << "_q: "<< _q.transpose() << endl<< endl;

}
void CController::CLIK()
{
	_x_des_left_hand(0) = _x_left_hand(0);							_x_des_right_hand(0) = _x_right_hand(0);
	// _x_des_left_hand(1) = 0.276268 + 0.15*cos(2*M_PI*_t*0.2);		_x_des_right_hand(1) = -0.276032 + 0.15*cos(-2*M_PI*_t*0.2);
	// _x_des_left_hand(2) = 0.697698 + 0.15*sin(2*M_PI*_t*0.2);		_x_des_right_hand(2) =  0.697698 + 0.15*sin(-2*M_PI*_t*0.2);


	_x_err_left_hand = _x_des_left_hand.head(3) - Model._x_left_hand;
	_R_des_left_hand = CustomMath::GetBodyRotationMatrix(_x_des_left_hand(3), _x_des_left_hand(4), _x_des_left_hand(5));
	_R_err_left_hand = -CustomMath::getPhi(Model._R_left_hand, _R_des_left_hand);

	_x_err_right_hand = _x_des_right_hand.head(3) - Model._x_right_hand;
	_R_des_right_hand = CustomMath::GetBodyRotationMatrix(_x_des_right_hand(3), _x_des_right_hand(4), _x_des_right_hand(5));
	_R_err_right_hand = -CustomMath::getPhi(Model._R_right_hand, _R_des_right_hand);

	_x_des_err.setZero();
	_x_des_err.segment(0, 3) = _x_err_left_hand;
	_x_des_err.segment(3, 3) = _R_err_left_hand;
	_x_des_err.segment(6, 3) = _x_err_right_hand;
	_x_des_err.segment(9, 3) = _R_err_right_hand;

	_xdot_des.segment(0,6) = _xdot_des_left_hand;
	_xdot_des.segment(6,6) = _xdot_des_right_hand;

	_Null_hands = _Id_15 - CustomMath::pseudoInverseQR(_J_hands)*_J_hands;

	Eigen::Matrix<double, 15, 1> _null_force;
	// _null_force = -_Null_hands *  (Model._avg_joint_position - _q);
	_null_force = -_Null_hands *  _qdot * _qdot;

	_qdot_des_clik =  CustomMath::pseudoInverseQR(_J_hands) *(_xdot_des + 20*_x_des_err) + 2*_null_force;
	_q_des_clik += _qdot_des_clik*_dt;

	_qddot_des = 400*(_q_des_clik - _q) + 40*(_qdot_des_clik - _qdot);
	_torque = Model._A * _qddot_des + Model._bg;


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
	_Null_hands = _Id_15 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();

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
	_torque = _J_T_hands * _Lambda_hands * _xddot_star +_Null_hands * Model._A * (400 * (_q_des - _q)+ 10*(- _qdot)) + Model._bg ;

	// for (int i=0; i<7; i++) 
	// {	
	// 	lstm_pred_compare(i) = _torque(i+1);
	// 	lstm_pred_compare(i+7) = K_obs(i+1);
	// 	lstm_pred_compare(i+14) = -_torque_observer(i+1);
	// }

	// fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/test1.txt",ios::app);
	// fout << lstm_pred_compare.transpose() <<endl;
	// fout.close();

}
void CController::MPC()
{
	// auto start = std::chrono::high_resolution_clock::now();
	if ( freq_check == loop){
		_mpc_init_t = _t;
		///////////////////////////////// set state matrix// X^T = [ q, qdot ]^T
		// A_d = 28 x 28
		// B_d = 28 x 14
		// 1. C_d = 14x28 == q  
		A_d.block<14,14>(0,0) = _Id_14;	A_d.block<14,14>(0,14) = dT * _Id_14;
										A_d.block<14,14>(14,14) = _Id_14;
		B_d.block<14,14>(14,0) = dT * _Id_14;
		// C_d.block<6,6>(0,7) = _Id_6;	C_d.block<6,6>(6,13) = _Id_6;	// y_k^T = [x, xdot]^T
		////////////////////////////////////////////////////////////////////////////////////////
		// defin F matrix
		F.block<28,28>(0,0) = A_d;
		for (int i = 1; i <Np; i++){
			F.block<28,28>(28*i,0) = F.block<28,28>(28*(i-1),0) * A_d;
		}
		// define Phi_temp matrix
		temp_phi.block<28,28>(0,0) = _Id_28;
		for (int i = 1; i <Np; i++){
			temp_phi.block<28,28>(28*i,0) = temp_phi.block<28,28>(28*(i-1),0) * A_d; // 7*Np x 14
		}
		// define Phi_temp_B matrix
		for (int i = 0; i <Np; i++){
			temp_phi_B.block<28,14>(28*i,0) = temp_phi.block<28,28>(28*i,0) * B_d; // 14 x 7
		}// // temp_phi_B = temp_phi * B_d;
		// define Phi matrix
		for (int i = 0; i <Np; i++){
			for (int j = 0; j <Np; j++){
				if( j<=i){
					Phi.block<28,14>(28*i, 14*j) = temp_phi_B.block<28,14>(28*(i-j),0);
				}
			}
		}
		Y_ref = Y_des;

		max_iter = 1000;
		//////////////////////////
		// X = 28 x 1 [q,qdot]^T
		X.segment(0,14) = _q.segment(1,14);
  		X.segment(14,14) = _qdot.segment(1,14);

		//////////////////////////
		// qp.InitializeProblemSize(Np*14, 0);
		qp.InitializeProblemSize(Np*14, Np*28);
		_H.setZero(qp._num_var, qp._num_var);
		_g.setZero(qp._num_var);
		_A.setZero(qp._num_cons, qp._num_var);
		_lbA.setZero(qp._num_cons);
		_ubA.setZero(qp._num_cons);
		_lb.setZero(qp._num_var);
		_ub.setZero(qp._num_var);
		// set cost function x^T*H*x + x^T*g
		_H.noalias() =  Phi.transpose()* Q * Phi + R;
		// _H1.noalias() =  Phi.transpose()* Q * Phi;
		_g =  2*Phi.transpose()*Q*(F*X + Phi*U_old - Y_ref);
		qp.UpdateMinProblem(_H,_g);

		_A = Phi;
		_eq_min_constraint = min_constraint - F*X - Phi*U_old;
		_eq_max_constraint = max_constraint - F*X - Phi*U_old;

		for (int i = 0; i < qp._num_cons; i++)
		{
			_lbA(i) = _eq_min_constraint(i); 
			_ubA(i) = _eq_max_constraint(i);
		}
		qp.UpdateSubjectToAx(_A, _lbA, _ubA);  
		// // torque limit
		for (int i = 0; i < qp._num_var; i++){ 
			_lb(i) =  -1000.0 ;
			_ub(i) =  1000.0 ;
		}
		qp.UpdateSubjectToX(_lb, _ub); 
		// //Solve
		qp.EnableEqualityCondition(0.0001);
		// qp.EnablePrintOptionDebug(); // qpOASES check debug
		qp.SolveQPoases(max_iter);
		_opt_u = qp._Xopt;			// jerk 
		U = U_old + _opt_u;			// ddot_q
		
		X_next = A_d * X + B_d * U.segment(0,14);
		
		for (int i=0; i<Np; i++){
			U_old.segment(14 * i,14) = U.segment(0,14);
		}

		freq_check=0;

	}
	freq_check++;

	_qdes_inter =  CustomMath::LinearInterpolation(X, X_next, _t, _mpc_init_t, _mpc_init_t+dT).segment(0,14);
	_qdotdes_inter =  CustomMath::LinearInterpolation(X, X_next, _t, _mpc_init_t, _mpc_init_t+dT).segment(14,14);

	_qdes_esti(0) = 0.0;
	_qdes_esti.segment(1,14) = _qdes_inter;
	_qdotdes_esti(0) = 0.0;
	_qdotdes_esti.segment(1,14) = _qdotdes_inter;
	_torque = Model._A*(400*(_qdes_esti - _q) + 40*(_qdotdes_esti - _qdot)) + Model._bg; 

	for (int i=0; i< 7; i++){
			log(i) = _q(i+1);
			log(i+7) = _q_des(i+1);
	}

	fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/test1.txt",ios::app);
	fout << log.transpose() <<endl;
	fout.close();
}

void CController::Initialize()
{	
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/lstm.txt");
	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/test1.txt");
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
	_diag_A.setZero(15,15);

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
	FT_left.setZero(6);
	FT_right.setZero(6);
	CT_Force.setZero(12);
	_pre_CT_force.setZero(12);
	_CT_force.setZero(12);
	CT_Torque.setZero(15);

	_x_error.setZero(12);
	_xdot_error.setZero(12);

	//OSF

	_J_hands.setZero(12, 15);
	_Jdot_hands.setZero(12, 15);
	_Jdot_qdot.setZero(12);
	_pre_J_hands.setZero(12, 15);
	_pre_Jdot_hands.setZero(12, 15);
	_J_T_hands.setZero(15, 12);
	_Lambda_hands.setZero(12, 12);
	_Null_hands.setZero(15, 15);
	_J_bar_T_hands.setZero(12, 15);

	_J_pos_hands.setZero(6, 15);
	_J_pos_T_hands.setZero(15, 6);
	_J_ori_hands.setZero(6, 15);
	_J_ori_T_hands.setZero(15, 6);
	_Lambda_pos_hands.setZero(6, 6);
	_Lambda_ori_hands.setZero(6, 6);
	_Null_hands_pos.setZero(15, 15);
	_Null_hands_ori.setZero(15, 15);

	_pos_goal_left_hand.setZero();
	_rpy_goal_left_hand.setZero();
	_pos_goal_right_hand.setZero();
	_rpy_goal_right_hand.setZero();

	_S_T.setZero(15, 6);
	_J_bar_T_hands_S_T.setZero(12,6);
	_W_mat_S.setZero(6, 6);
	_J_tilde_T.setZero(6,12);
	_xddot_reinforce.setZero(12);
	_torque_reinforce.setZero(15);


	_Id_15.setIdentity(15, 15);
	_Id_14.setIdentity(14,14);
	_Id_12.setIdentity(12, 12);
	_Id_3.setIdentity(3,3);
	_Id_4.setIdentity(4,4);
	_Id_8.setIdentity(8,8);


	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_zero_vec.setZero(_dofj);

	JointTrajectory.set_size(15);
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

	_Pddot_des.setZero();
	_wdot_des.setZero();
	_R_err.setZero();
	_gravity.setZero();
	_gravity << 0, 0, 9.80665;
	_b.setZero();
	
	log.setZero(14);
}
bool CController::ParticleYamlRead(std::string path, Eigen::VectorXd& unit)
{
    YAML::Node yaml_node;
    try
    {
        yaml_node = YAML::LoadFile(path.c_str());
        unit(0) = yaml_node["MPC_ddot_q"]["q"].as<double>();
		unit(1) = yaml_node["MPC_ddot_q"]["qdot"].as<double>();
        unit(2) = yaml_node["MPC_ddot_q"]["R"].as<double>();
		unit(3) = yaml_node["MPC_ddot_q"]["dT"].as<double>();
        unit(4) = yaml_node["MPC_ddot_q"]["Np"].as<double>();
		unit(5) = yaml_node["MPC_ddot_q"]["loop"].as<double>();
	

    }
    catch(const std::exception& e)
    {
        std::cerr << "fail to read particle yaml file" <<std::endl;
        return false;
    }
    return true;
}
void CController::Initialize_MPC()
{
	Eigen::VectorXd param;
	param.setZero(6);
    ParticleYamlRead("../libs/data/param.yaml", param);

	dT = param(3);
	Np = param(4);
	loop = param(5);
	freq_check = loop;

	cout <<"dT : "<<dT<<endl;
	cout <<"Np : "<<Np<<endl;
	cout <<"loop : "<<loop<<endl;

	_state_size = (_dofj-1)*2; // _dofj = 15 / total : 28 -> q: 14, qdot: 14
	X.setZero(_state_size);
	X_next.setZero(_state_size);

	A_d.setZero(_state_size, _state_size);
	B_d.setZero(_state_size, 14);

	// C_d.setZero(12,_state_size); 

	F.setZero(_state_size*Np, _state_size);
	temp_phi.setZero(_state_size*Np, _state_size);
	temp_phi_B.setZero(_state_size*Np, 14);
	Phi.setZero(_state_size*Np, 14*Np);


	Q.setZero(_state_size*Np, _state_size*Np);
	Q_temp.setZero(_state_size, _state_size);

	for(int i=0; i<14; i++){
		Q_temp(i,i) = param(0);				// q
		Q_temp(i+14,i+14) = param(1);			// qdot
	}
	for(int i=0; i<Np; i++){
		Q.block<28,28>(28*i, 28*i) = Q_temp; 
	}
	cout << Q_temp <<endl<<endl;
	R.setZero(14*Np, 14*Np);
	for(int i=0; i<14*Np; i++){
		R(i,i) = param(2);
	}  
	cout << R.block<14,14>(0,0) <<endl<<endl;
	Y_temp.setZero(Np*28);
	Y_des.setZero(Np*28);
	Y_ref.setZero(Np*28);// Y_ref is used to calculate MPC costs 

	_const_min.setZero(28);
	_const_max.setZero(28);
	_const_min.segment(0,14) = Model._min_joint_position.segment(1,14); 	// _min_joint_position
	_const_min.segment(14,14) = Model._min_joint_velocity.segment(1,14);	// _min_joint_velocity

	_const_max.segment(0,14) = Model._max_joint_position.segment(1,14);	// _max_joint_position
	_const_max.segment(14,14) = Model._max_joint_velocity.segment(1,14);	// _max_joint_velocity

	min_constraint.setZero(28*Np);
	max_constraint.setZero(28*Np);			
	_eq_min_constraint.setZero(28*Np);
	_eq_max_constraint.setZero(28*Np);

	for(int i=0; i<Np; i++)
	{
		min_constraint.segment(i*28,28) = _const_min;
		max_constraint.segment(i*28,28) = _const_max;
	}

	U_old.setZero((_dofj-1)*Np);
	U.setZero((_dofj-1)*Np);
	_opt_u.setZero((_dofj-1)*Np);

	// _opt_u.setZero(14);

	_qdes_inter.setZero(14);
	_qdotdes_inter.setZero(14);
	_qdes_esti.setZero(15);
	_qdotdes_esti.setZero(15);

	Y_temp.setZero();

	b_100hz_t = true;

	_Id_28.setIdentity(28,28);

}


