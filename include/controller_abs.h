#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <rbdl.h>

#include <fstream>

#include "trajectory.h"
#include "robotmodel.h"
#include "custommath.h"
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <thread>


#define InputNOF 2
#define EACH_DOF 7
//#define DEG2RAD (0.01745329251994329576923690768489)
//#define RAD2DEG 1.0/DEG2RAD

using namespace std;
using namespace Eigen;

class CController
{

public:
	CController(int JDOF);
	virtual ~CController();	
	void read(double time, double* q, double* qdot);
	void write(double* torque);
	void control_mujoco();
	VectorXd RPY();

	CModel Model;

public:

	std::mutex calculation_mutex_;
	std::thread async_calculation_thread_;
	
	VectorXd _torque; //15
	VectorXd pre_torque;
	VectorXd _q; //joint angle vector
	VectorXd _qdot; //joint velocity vector
	VectorXd _Fd, _Fd_error, mujoco_qddot;

	VectorXd _q_co; //14
	VectorXd _qdot_co; //14
	MatrixXd _dig_A_co;

private:
	double _t;
	bool _bool_init;
	double _init_t;
	double _dt;
	double _pre_t;
	int _dofj; //number of joint
	int _control_mode; //1: joint space, 2: operational space 
	double friction_coeff;
	VectorXd _zero_vec;	
	VectorXd _q_home, _q_home_co, _q_home_co_init;

	VectorXd _pre_q, _ttorque;
	VectorXd _pre_qdot;

	void Initialize();

	//plan
	void reset_target(double motion_time, VectorXd target_joint_position);
	void reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh, Vector3d target_pos_rh, Vector3d target_ori_rh);

	void reset_targetAbs(double motion_time, Eigen::Matrix<double, 6, 1> target_pose_abs, Eigen::Matrix<double, 6, 1> target_pose_rel);

	void motionPlan();

	double y_left, y_right;
	int _cnt_plan;
	VectorXd _time_plan;
	VectorXi _bool_plan;

	//controller
	double _kpj; //joint control P gain
	double _kdj; //joint control D gain	
	double _kp; //Operational space control P gain
	double _kd; //Operational space control D gain	
	double _ki;

	double _kp_c0;
	
	MatrixXd Md;
	MatrixXd Md_I;
	MatrixXd Bd; // when, Critical damped .. B = 2 * sqrt(MK) 
	MatrixXd Kd;
	MatrixXd _lamda_object;

	void JointControl();
	void OperationalSpaceControl();


	void CLIK();
	void Wrist_IK(double p1, double p2);

	void TEST_control();

	double p1, p2;
	VectorXd x_goal_co, x_curr_co, xdot_curr_co;
	VectorXd x_goal_l_co, x_goal_r_co;
	VectorXd x_curr_l_co, x_curr_r_co;
	VectorXd pre_x_des_L, pre_x_des_R;
	VectorXd pre_qpos;
	//robot model
	
	void ModelUpdate();
	MatrixXd _J_hands; // 12x15 
	MatrixXd _J_T_hands; // 15x12
	MatrixXd _J_bar_T_hands;//12x15
	MatrixXd _Jdot_hands;
	MatrixXd _pre_J_hands;
	MatrixXd _pre_Jdot_hands;
	VectorXd _Jdot_qdot;

	MatrixXd _J_pos_hands; // 6x15
	MatrixXd _J_pos_T_hands; // 15x6
	MatrixXd _J_ori_hands; // 6x15
	MatrixXd _J_ori_T_hands; // 15x6	
	
	MatrixXd _J_hands_co; // 12x14
	MatrixXd _J_T_hands_co; // 14x12
	MatrixXd _J_bar_T_hands_co; // 12x14
	MatrixXd _J_pos_hands_co; // 6x14
	MatrixXd _J_pos_T_hands_co; // 14x6
	MatrixXd _J_ori_hands_co; // 6x14
	MatrixXd _J_ori_T_hands_co; // 14x6	

	VectorXd _x_left_hand_co; //state
	VectorXd _x_right_hand_co; //state
	VectorXd _xdot_left_hand_co; //state
	VectorXd _xdot_right_hand_co; //state

	VectorXd _x_des_err_co;// 12
	VectorXd _xdot_des_co,_x_des_LR, _x_curr_LR;// 12

	//operational space variables (two hand)
	VectorXd _x_left_hand; //state
	VectorXd _x_right_hand; //state
	VectorXd _xdot_left_hand; //state
	VectorXd _xdot_right_hand; //state
	MatrixXd _Lambda_hands; //inertia matri 12x12
	MatrixXd _Null_hands; //null space projection matrix 15x15	
	MatrixXd _Id_28,_Id_15, _Id_14, _Id_12,_Id_3,_Id_4,_Id_8;
	VectorXd _xddot_star; //12
	VectorXd _x_error; // 12x1 x error x,y,z,r,p,y
	VectorXd _xdot_error;// 12x1 xdot error x,y,z,r,p,y
	
	MatrixXd _Lambda_pos_hands; //inertia matri 6x6
	MatrixXd _Lambda_ori_hands; //inertia matri 6x6
	MatrixXd _Null_hands_ori; //null space projection matrix 15x15
	MatrixXd _Null_hands_pos; //null space projection matrix 15x15

	MatrixXd _S_T; //selection matrix transpose
	MatrixXd _J_bar_T_hands_S_T; //
	MatrixXd _W_mat_S;
	MatrixXd _J_tilde_T; //J^T with consideration of selection matrix
	VectorXd _xddot_reinforce; //for additonal feedback acceleration
	VectorXd _torque_reinforce;

	Vector3d _x_err_left_hand;
	Vector3d _x_err_right_hand;
	Vector3d _xdot_err_left_hand;
	Vector3d _xdot_err_right_hand;
	Vector3d _R_err_left_hand;
	Vector3d _R_err_right_hand;
	Vector3d _Rdot_err_left_hand;
	Vector3d _Rdot_err_right_hand;

	// Sensor
	VectorXd FT_left; // 1x6
	VectorXd FT_right; // 1x6
	VectorXd CT_Force; // 12x1 Contact Force at FT Sensor 
	VectorXd _pre_CT_force;
	VectorXd _CT_force; // 12x1 lowpass filter Contact Force
	VectorXd CT_Torque; // 15x1 Contact Torque at FT Sensor

	
	//motion trajectory
	double _start_time, _end_time, _motion_time;


	//joint space
	bool _bool_joint_motion;
	CTrajectory JointTrajectory; //size = joint dof
	VectorXd _q_goal;
	VectorXd _qdot_goal;
	VectorXd _q_des;//desired joint angle vector
	VectorXd _qdot_des;//desired joint velocity vector

	VectorXd _q_des_co, input_q;//desired joint angle vector
	VectorXd _qdot_des_co, _pre_qdot_des_co;//desired joint velocity vector
	VectorXd _q_goal_co;
	VectorXd _qdot_goal_co;

	double la,b;
	//operational space (two hand)
	bool _bool_ee_motion;
	CTrajectory RightHandTrajectory; //size = 6
	CTrajectory LeftHandTrajectory; //size = 6

	HTrajectory absTrajectory; //size = 6
	HTrajectory relTrajectory; //size = 6
	
	Eigen::Matrix<double, 6, 1> p_a_des, p_r_des;
	Eigen::Matrix<double, 6, 1> pdot_a_des, pdot_r_des;
	Eigen::Matrix<double, 3, 3> R_a_des, R_r_des;

	VectorXd _x_goal_left_hand;
	VectorXd _xdot_goal_left_hand;
	VectorXd _x_des_left_hand;
	VectorXd _xdot_des_left_hand;
	VectorXd _x_goal_right_hand;
	VectorXd _xdot_goal_right_hand;
	VectorXd _x_des_right_hand;
	VectorXd _xdot_des_right_hand;
	Matrix3d _R_des_left_hand;
	Matrix3d _R_des_right_hand;

	VectorXd error_f , we;
	Matrix3d x_zero;
	Matrix3d R_left_current, R_right_current;
 	VectorXd _R_left_current,_R_right_current;

	Vector3d _pos_goal_left_hand;
	Vector3d _rpy_goal_left_hand;
	Vector3d _pos_goal_right_hand;
	Vector3d _rpy_goal_right_hand;

	// addons  ////////////////////////////////////////////////////////////////////////////////////////	
	bool _bool_safemode;

	double _dist_shoulder_hand_left;
	double _dist_shoulder_hand_right;
	double _workspace_avoid_gain;
	Vector3d _dir_hand_to_shoulder_left;
	Vector3d _dir_hand_to_shoulder_right;
	Vector3d _acc_workspace_avoid_left;
	Vector3d _acc_workspace_avoid_right;

	MatrixXd _diag_A;
	////////////////////////////////////////////////////////////////////////////////////////////////////
	
	ofstream fout;
	VectorXd NN_output, NN_output_j1, NN_Fext, stack4;
	int check10, check100;
	bool _check100;
	MatrixXd test_nn;

	int i;

	VectorXd sensor_qpos, sensor_qvel;
	MatrixXd J_Rhand_j4, J_Rhand_j4_inv;
	double _pitch, _yaw;
	double _t_emp;

	VectorXd pos_error, pre_qddddd, pre_qddddd_dot, qd, qdotd, _q_des0;

	double log_output[2000000];
	int check_14;

	VectorXd constt;
	MatrixXd diag_new_A;
	
	////////////////
	void velocity_observer();
	int k_p_obs, k_d_obs, _kvj;
	VectorXd _q_obs_err, _q_obs ;
	VectorXd _qdot_obs_err, _qdot_obs;
	VectorXd _torque_nominal;
	VectorXd _torque_maintask;
	VectorXd _est_qddot;

	VectorXd _torque_dist;

	MatrixXd inv_M_obs;
	VectorXd K_obs;
	////////////////

	VectorXd log;
	bool ParticleYamlRead(std::string path, Eigen::VectorXd& unit); // for yaml

	void MPC();
	void Initialize_MPC();
	double freq_check, loop;
	double _mpc_init_t;
	double dT;
	double Np;
	double _state_size;

	MatrixXd A_d;
	MatrixXd B_d;
	MatrixXd C_d;

	MatrixXd F;
	MatrixXd temp_phi;
	MatrixXd temp_phi_B;
	MatrixXd Phi;

	VectorXd Y_ref, Y_des, Y_temp;
	VectorXd X;
	VectorXd X_next;

	MatrixXd Q_temp;
    MatrixXd Q_temp_last;
    MatrixXd Q;
    MatrixXd R;

	void Leader_Follower();
	void reset_target(double motion_time, Vector3d target_pos_leader, Vector3d target_ori_leader);
	HTrajectory LeaderHandTrajectory; // task space trajectory
	HTrajectory FollowerHandTrajectory; // task space trajectory

	Eigen::Matrix<double, 6, 1> _x_goal_leader_hand;
	Eigen::Matrix<double, 6, 1> _xdot_goal_leader_hand;

	Eigen::Matrix<double, 3, 1> _pos_goal_leader_hand;
	Eigen::Matrix<double, 3, 1> _rpy_goal_leader_hand;

	Eigen::Matrix<double, 6, 1> _x_goal_follower_hand;
	Eigen::Matrix<double, 6, 1> _xdot_goal_follower_hand;

	Eigen::Matrix<double, 3, 1> _pos_goal_follower_hand;
	Eigen::Matrix<double, 3, 1> _rpy_goal_follower_hand;

	Eigen::Matrix<double, 3, 3> _Rl_des_hand;
	Eigen::Matrix<double, 3, 3> _Rr_des_hand;

	// relative vector values
	Eigen::Matrix<double, 3, 1> _x_rel;
	Eigen::Matrix<double, 3, 1> _x_rel_ori;
	Eigen::Matrix<double, 3, 3> _R_rel;
	Eigen::Matrix<double, 3, 3> _R_rel_des_init;

	Eigen::Matrix<double, 3, 1> _x_rel_pos;
	Eigen::Matrix<double, 3, 1> _x_rel_pos_des;
	Eigen::Matrix<double, 3, 1> _x_rel_d_pos;
	Eigen::Matrix<double, 3, 1> _x_rel_d_ori;
	Eigen::Matrix<double, 6, 1> _x_rel_d;
	Eigen::Matrix<double, 6, 1> _x_right_error_hand;
	Eigen::Matrix<double, 6, 1> _x_left_error_hand;

	double _k_rel_ori, _k_rel_pos ;
	Eigen::Matrix<double, 7, 7> M_l;
	Eigen::Matrix<double, 7, 7> M_r;
	Eigen::Matrix<double, 7, 7> M_l_diag;
	Eigen::Matrix<double, 7, 7> M_r_diag;

	Eigen::Matrix<double, 6, EACH_DOF> _J_l_hand;
	Eigen::Matrix<double, 6, EACH_DOF> _J_r_hand;

	Eigen::Matrix<double, EACH_DOF, 6> _J_bar_right_hand;
	Eigen::Matrix<double, EACH_DOF, 6> _J_bar_left_hand;
	Eigen::Matrix<double, EACH_DOF, 1> _qdot_right_des_cl;
	Eigen::Matrix<double, EACH_DOF, 1> _q_right_des_cl;

	Eigen::Matrix<double, EACH_DOF, 1> _qdot_left_des_cl;
	Eigen::Matrix<double, EACH_DOF, 1> _q_left_des_cl;

	VectorXd _qddot_des;

	// Null mapping
	Eigen::Matrix<double, EACH_DOF, EACH_DOF> Null_right;
	Eigen::Matrix<double, 7, 7> _Id_7;
	Eigen::Matrix<double, EACH_DOF, 1> _del_H_r;
	Eigen::Matrix<double, EACH_DOF, 1> _q_r_null_des;
	Eigen::Matrix<double, EACH_DOF, 1> _qdot_r_null;

	Eigen::Matrix<double, EACH_DOF, EACH_DOF> Null_left;
	Eigen::Matrix<double, EACH_DOF, 1> _del_H_l;
	Eigen::Matrix<double, EACH_DOF, 1> _q_l_null_des;
	Eigen::Matrix<double, EACH_DOF, 1> _qdot_l_null;
	bool rot_update = true;
	
	Eigen::Matrix<double, 6, 6> _Lambda_r;
	Eigen::Matrix<double, 6, 6> _Lambda_l;

	void AbsoluteFrameController();
	Eigen::Matrix<double, 6, 1> _x_abs;
	Eigen::Matrix<double, 6, 1> _xdot_abs;

	Eigen::Matrix<double, 3, 3> _R_abs;

	Eigen::Matrix<double, 6, 14> _J_a;
	Eigen::Matrix<double, 6, 14> _J_r;

	void reset_target1(double motion_time, VectorXd target_pos, VectorXd target_ori);
	Eigen::Matrix<double, 6, 1> _x_goal_abs;
	Eigen::Matrix<double, 6, 1> _xdot_goal_abs;
	Eigen::Matrix<double, 3, 1> _pos_goal_abs;
	Eigen::Matrix<double, 3, 1> _rpy_goal_abs;

	HTrajectory AbsTrajectory; // task space trajectory
	Eigen::Matrix<double, 6, 1> _x_des_abs;
	Eigen::Matrix<double, 3, 3> _R_des_abs;
	Eigen::Matrix<double, 6, 1> _xdot_des_abs;

	Eigen::Matrix<double, 14, 1> _nominal_qdot;
	Eigen::Matrix<double, 12, 14> _J;
	Eigen::Matrix<double, 12, 1> _v_k;

	Eigen::Matrix<double, 6, 1> _x_abs_error;
	Eigen::Matrix<double, 6, 1> _xdot_abs_error;

	Eigen::Matrix<double, 14, 1> _nominal_q;

	Eigen::Matrix<double, 14, 14> _Null_J;
	Eigen::Matrix<double, 14, 1> _Null_qdot;

	Eigen::Matrix<double, 3, 3> _Skew_w;
	Eigen::Matrix<double, 3, 1> _xdot_r_des;
	Eigen::Matrix<double, 6, 1> _abs_x_abs;
	Eigen::Matrix<double, 6, 1> _abs_xdot_abs;


	Eigen::Matrix<double, 3, 1> p_r, p_a;
	Eigen::Matrix<double, 3, 3> R_r, R_a;
	Eigen::Matrix<double, 3, 3> Rdot_a;
	Eigen::Matrix<double, 6, 1> p_r_pose, p_a_pose;
	Eigen::Matrix<double, 6, 1> pdot_r_pose, pdot_a_pose;

	Eigen::Matrix<double, 3, 1> pdot_r, pdot_a;
	Eigen::Matrix<double, 3, 1> omega_r, omega_a;

	Eigen::Matrix<double, 6, 2*EACH_DOF> J_a, J_r;
	Eigen::Matrix<double, 12, 2*EACH_DOF> J_ar;

	Eigen::Matrix<double, 6, 1> e_a, e_r;
	Eigen::Matrix<double, 6, 1> v_a, v_r;
	Eigen::Matrix<double, 12, 1> e_ar, v_des;

	Eigen::Matrix<double, 6, 1> _x_goal_rel;
	Eigen::Matrix<double, 6, 1> _xdot_goal_rel;

	Eigen::Matrix<double, 6, 1> _abs_goal, _rel_goal;

	Eigen::Matrix<double, 12, 1> _qdot_ar, _q_ar;
	
	void AbsIK();
	
};

#endif
