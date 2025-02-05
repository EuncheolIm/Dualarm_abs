#include "robotmodel.h"
#define JDOF 15


CModel::CModel()
{
	Initialize();
}

CModel::~CModel()
{
}

void CModel::Initialize()
{
	_bool_model_update = false;
	_bool_kinematics_update = false;
	_bool_dynamics_update = false;
	_bool_Jacobian_update = false;

	_k = JDOF;
	_k_co = JDOF;

	_q_co.setZero(_k_co);
	_qdot_co.setZero(_k_co);
	_zero_vec_joint_co.setZero(_k_co);

	_max_joint_torque.setZero(_k);
	_min_joint_torque.setZero(_k);
	_max_joint_velocity.setZero(_k);
	_min_joint_velocity.setZero(_k);
	_max_joint_position.setZero(_k);
	_min_joint_position.setZero(_k);
	_max_ctrl_joint_torque.setZero(_k);
	_min_ctrl_joint_torque.setZero(_k);

	_A_co.setZero(_k_co,_k_co);
	_g_co.setZero(_k_co);
	_b_co.setZero(_k_co);
	_bg_co.setZero(_k_co);

	_position_local_zerovec.setZero();

	_global_rotate.setZero(3, 3);	

	_id_left_hand_co = 8;//7
	_id_right_hand_co = 15;

	_J_tmp_co.setZero(6,_k_co);
	_J_left_hand_co.setZero(6,_k_co);
	_J_left_hand_pos_co.setZero(3,_k_co);
	_J_left_hand_ori_co.setZero(3,_k_co);
	_x_left_hand_co.setZero();
	_R_left_hand_co.setZero();
	_xdot_left_hand_co.setZero(6);

	_J_right_hand_co.setZero(6,_k_co);
	_J_right_hand_pos_co.setZero(3,_k_co);
	_J_right_hand_ori_co.setZero(3,_k_co);
	_x_right_hand_co.setZero();
	_R_right_hand_co.setZero();
	_xdot_right_hand_co.setZero(6);

	set_robot_config();
	load_model();
}

void CModel::load_model()
{
	//read urdf model
	RigidBodyDynamics::Addons::URDFReadFromFile("../model_backup/im_test.urdf", &_model, false, true);	
	// RigidBodyDynamics::Addons::URDFReadFromFile("../model/urdf/dual_arm/dualarm.urdf", &_model, false, true);	

	//body id 1: body_link (trunk)
	//body id 8: LWrR_Link (left hand)
	//body id 15: RWrR_Link (right hand)

	cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
	if (_model.dof_count != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

	_global_rotate.setIdentity(); //x������ �յ� ������ �ǵ��� �����ϱ� ���Ͽ�...	
	// _global_rotate = CustomMath::GetBodyRotationMatrix(0.0, 0.0, -90.0 * DEG2RAD);

	_bool_model_update = true; //check model update

	cout << "Model Loading Complete." << endl << endl;
}

void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q_co = q;
	_qdot_co = qdot;

	if (_bool_model_update == true)
	{
		RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q_co, &_qdot_co, NULL); //update kinematics
	}
	else
	{
		cout << "Robot model is not ready. Please load model first." << endl << endl;
	}
	_bool_kinematics_update = true; //check kinematics update
}

void CModel::update_dynamics()
{
	if (_bool_kinematics_update == true)
	{
		RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q_co, _A_co, false); //update dynamics
		RigidBodyDynamics::InverseDynamics(_model, _q_co, _zero_vec_joint_co, _zero_vec_joint_co, _g_co, NULL); //get _g
		RigidBodyDynamics::InverseDynamics(_model, _q_co, _qdot_co, _zero_vec_joint_co, _bg_co, NULL); //get _g+_b
		_b_co = _bg_co - _g_co; //get _b
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
	_bool_dynamics_update = true; //check kinematics update
}

void CModel::calculate_EE_Jacobians()
{
	if (_bool_kinematics_update == true)
	{
		_J_left_hand_co.setZero();
		_J_tmp_co.setZero();
	
		RigidBodyDynamics::CalcPointJacobian6D(_model, _q_co, _id_left_hand_co, _position_local_task_left_hand, _J_tmp_co, false); //left hand
		// _J_left_hand_co.block<3, JDOF>(0, 0) = _global_rotate *_J_tmp_co.block<3, JDOF>(3, 0);
		// _J_left_hand_co.block<3, JDOF>(3, 0) = _global_rotate *_J_tmp_co.block<3, JDOF>(0, 0);
		// _J_left_hand_pos_co = _global_rotate * _J_tmp_co.block<3, JDOF>(3, 0);
		// _J_left_hand_ori_co = _global_rotate * _J_tmp_co.block<3, JDOF>(0, 0);
		_J_left_hand_co.block<3, JDOF>(0, 0) = _J_tmp_co.block<3, JDOF>(3, 0);
		_J_left_hand_co.block<3, JDOF>(3, 0) = _J_tmp_co.block<3, JDOF>(0, 0);
		_J_left_hand_pos_co = _J_tmp_co.block<3, JDOF>(3, 0);
		_J_left_hand_ori_co = _J_tmp_co.block<3, JDOF>(0, 0);

		_J_right_hand_co.setZero();
		_J_tmp_co.setZero();
		RigidBodyDynamics::CalcPointJacobian6D(_model, _q_co, _id_right_hand_co, _zero_vec_joint_co, _J_tmp_co, false); //right hand		
		// _J_right_hand_co.block<3, JDOF>(0, 0) = _global_rotate * _J_tmp_co.block<3, JDOF>(3, 0);
		// _J_right_hand_co.block<3, JDOF>(3, 0) = _global_rotate * _J_tmp_co.block<3, JDOF>(0, 0);
		// _J_right_hand_pos_co = _global_rotate * _J_tmp_co.block<3, JDOF>(3, 0);
		// _J_right_hand_ori_co = _global_rotate * _J_tmp_co.block<3, JDOF>(0, 0);
		_J_right_hand_co.block<3, JDOF>(0, 0) = _J_tmp_co.block<3, JDOF>(3, 0);
		_J_right_hand_co.block<3, JDOF>(3, 0) = _J_tmp_co.block<3, JDOF>(0, 0);
		_J_right_hand_pos_co = _J_tmp_co.block<3, JDOF>(3, 0);
		_J_right_hand_ori_co = _J_tmp_co.block<3, JDOF>(0, 0);

		_bool_Jacobian_update = true;
		// cout<<endl;
		// cout<< "_J_left_hand  IM: "<<endl;
		// for(int i=0; i<6; i++)
		// {
		// 	for(int j=0; j<7; j++)
		// 	{
		// 		cout << _J_left_hand_co(i,j) << " ";
		// 	}
		// 	cout<<endl;
		// }
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_positions_orientations()
{
	if (_bool_kinematics_update == true)
	{
		// _x_left_hand_co.setZero();
		// _x_left_hand_co = _global_rotate * RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q_co, _id_left_hand_co, _position_local_task_left_hand, false);
		// _x_right_hand_co.setZero();
		// _x_right_hand_co = _global_rotate * RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q_co, _id_right_hand_co, _position_local_task_right_hand, false);

		// _R_left_hand_co = _global_rotate*(RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q_co, _id_left_hand_co, false).transpose());
		// _R_right_hand_co = _global_rotate*(RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q_co, _id_right_hand_co, false).transpose());
		////////////////////////////////////////////////////////////////////////
		_x_left_hand_co.setZero();
		_x_left_hand_co = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q_co, _id_left_hand_co, _position_local_task_left_hand, false);
		_x_right_hand_co.setZero();
		_x_right_hand_co = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q_co, _id_right_hand_co, _position_local_task_right_hand, false);

		_R_left_hand_co = (RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q_co, _id_left_hand_co, false).transpose());
		_R_right_hand_co = (RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q_co, _id_right_hand_co, false).transpose());

	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_velocity()
{
	if (_bool_Jacobian_update == true)
	{

		_xdot_left_hand_co = _J_left_hand_co * _qdot_co;
		_xdot_right_hand_co = _J_right_hand_co * _qdot_co;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}
void CModel::set_robot_config()
{
	_position_local_task_left_hand.setZero();
	_position_local_task_left_hand(0) = -0.017;
	_position_local_task_left_hand(1) = -0.08;

	_position_local_task_right_hand.setZero();
	_position_local_task_right_hand(0) = -0.017;
	_position_local_task_right_hand(1) = -0.08;
	

	// _max_joint_torque(0) = 400.0;
	// _max_joint_torque(1) = 85.8;
	// _max_joint_torque(2) = 85.8;
	// _max_joint_torque(3) = 76.8;
	// _max_joint_torque(4) = 76.8;
	// _max_joint_torque(5) = 76.8;
	// _max_joint_torque(6) = 21.8;
	// _max_joint_torque(7) = 21.8;
	// _max_joint_torque(8) = 85.8;
	// _max_joint_torque(9) = 85.8;
	// _max_joint_torque(10) = 76.8;
	// _max_joint_torque(11) = 76.8;
	// _max_joint_torque(12) = 76.8;
	// _max_joint_torque(13) = 21.8;
	// _max_joint_torque(14) = 21.8;
	// _min_joint_torque = -_max_joint_torque;

	// _max_joint_velocity(0) = 0.05;
	// _max_joint_velocity(1) = 7.0;
	// _max_joint_velocity(2) = 7.0;
	// _max_joint_velocity(3) = 5.4;
	// _max_joint_velocity(4) = 5.4;
	// _max_joint_velocity(5) = 5.4;
	// _max_joint_velocity(6) = 14.0;
	// _max_joint_velocity(7) = 14.0;
	// _max_joint_velocity(8) = 7.0;
	// _max_joint_velocity(9) = 7.0;
	// _max_joint_velocity(10) = 5.4;
	// _max_joint_velocity(11) = 5.4;
	// _max_joint_velocity(12) = 5.4;
	// _max_joint_velocity(13) = 14.0;
	// _max_joint_velocity(14) = 14.0;
	// _min_joint_velocity = -_max_joint_velocity;

	// _max_joint_position(0) = 0.35;
	// _min_joint_position(0) = -0.35; //-0.6
	// _max_joint_position(1) = 90.0 * DEG2RAD;
	// _min_joint_position(1) = -90.0 * DEG2RAD;
	// _max_joint_position(2) = 90.0 * DEG2RAD;
	// _min_joint_position(2) = -15.0 * DEG2RAD;
	// _max_joint_position(3) = 90.0 * DEG2RAD;
	// _min_joint_position(3) = -90.0 * DEG2RAD;
	// _max_joint_position(4) = 120.0 * DEG2RAD;
	// _min_joint_position(4) = -30.0 * DEG2RAD;
	// _max_joint_position(5) = 90.0 * DEG2RAD;
	// _min_joint_position(5) = -90.0 * DEG2RAD;
	// _max_joint_position(6) = 90.0 * DEG2RAD;
	// _min_joint_position(6) = -90.0 * DEG2RAD;
	// _max_joint_position(7) = 45.0 * DEG2RAD;
	// _min_joint_position(7) = -45.0 * DEG2RAD;
	// _max_joint_position(8) = 90.0 * DEG2RAD;
	// _min_joint_position(8) = -90.0 * DEG2RAD;
	// _max_joint_position(9) = 15.0 * DEG2RAD;
	// _min_joint_position(9) = -90.0 * DEG2RAD;
	// _max_joint_position(10) = 90.0 * DEG2RAD;
	// _min_joint_position(10) = -90.0 * DEG2RAD;
	// _max_joint_position(11) = 30.0 * DEG2RAD;
	// _min_joint_position(11) = -120.0 * DEG2RAD;
	// _max_joint_position(12) = 90.0 * DEG2RAD;
	// _min_joint_position(12) = -90.0 * DEG2RAD;
	// _max_joint_position(13) = 90.0 * DEG2RAD;
	// _min_joint_position(13) = -90.0 * DEG2RAD;
	// _max_joint_position(14) = 45.0 * DEG2RAD;
	// _min_joint_position(14) = -45.0 * DEG2RAD;
	// -14.9 ~ 14.9

}