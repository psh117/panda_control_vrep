#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmController
{
public:
    enum ARM_CONTROL_MODE
    {
		NONE,
		HOME_JOINT_CTRL,
		INIT_JOINT_CTRL,
		SIMPLE_JACOBIAN,
		FEEDBACK_JACOBIAN,
		CLIK,
		CLIK_WITH_WEIGHTED_PSEUDO_INV,

		// TODO: implement your own mode
		
    };

private:
    size_t dof_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value

	// Task space
	Vector3d x_init_;
	Vector3d x_;
	Matrix3d rotation_;
	Matrix3d rotation_init_;
	Vector3d phi_;
	Vector6d x_dot_; // 6D (linear + angular)
	Vector6d x_error_; 

	// For controller
	Matrix<double,3,7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage 
	
	VectorXd q_temp_;	// For RBDL bug
	MatrixXd j_temp_;	// For RBDL bug

	Vector7d q_cubic_;
	Vector7d q_target_;

	Vector3d x_cubic_;
	Vector3d x_cubic_old_;
	Vector3d x_target_;

    unsigned long tick_;
    double play_time_;
    double hz_;
    double control_start_time_;

    ARM_CONTROL_MODE control_mode_;
    bool is_mode_changed_;




	// for robot model construction
    Math::Vector3d com_position_[DOF];
    Vector3d joint_posision_[DOF];

    shared_ptr<Model> model_;
    unsigned int body_id_[DOF];
    Body body_[DOF];
    Joint joint_[DOF];


	ofstream simple_jacobian_file_;
	ofstream feedback_jacobian_file_;
	ofstream clik_file_;
	ofstream clik_weight_file_;

private:
	void initFileDebug();
    void printState();
	void moveJointPosition(const Vector7d &target_pos, double duration);

public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData(const Vector7d &position, const Vector7d &velocity);
	void writeData(Vector7d &position, Vector7d &velocity, Vector7d &torque);
	void writeData(Vector7d &position);

public:
		ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_(NONE), is_mode_changed_(false)
	{
			initDimension(); initModel(); initFileDebug();
	}


    void setMode(ARM_CONTROL_MODE mode);
    void initDimension();
    void initModel();
    void initPosition();
    void compute();
};

#endif
