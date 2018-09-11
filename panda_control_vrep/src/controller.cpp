#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>



void ArmController::compute()
{
	// Kinematics calculation ------------------------------
	q_temp_ = q_;
	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, NULL, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], true);
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, true);
	for (int i = 0; i<2; i++)
		j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
	// -----------------------------------------------------
	j_v_ = j_.block < 3, DOF>(0, 0);

	if (is_mode_changed_)
	{
		is_mode_changed_ = false;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();

		x_init_ = x_;
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
	}

	switch (control_mode_)
	{
		// TODO: implement this
	case NONE:
		break;
	case HOME_JOINT_CTRL:
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		moveJointPosition(target_position, 5.0);
		break;
	}
	case INIT_JOINT_CTRL:
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI/2., 0.0, 0.0, 0.0;
		moveJointPosition(target_position, 5.0);
		break;
	}
	case SIMPLE_JACOBIAN:
	{
		Vector3d x_desired;
		Vector3d x_dot_desired;
		double duration = 5.0;
		x_desired << 0.6, 0.2, 0.1;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		Matrix<double, 7, 3> j_v_inverse;
		j_v_inverse = j_v_.transpose() * (j_v_ * j_v_.transpose()).inverse();

		x_dot_desired = (x_cubic_ - x_cubic_old_);

		x_cubic_old_ = x_cubic_;
		q_desired_ = q_ + j_v_inverse * x_dot_desired;

		if (play_time_ <= control_start_time_ + 7.0)
		{
			for (int i = 0; i < 7; i++)
			{
				simple_jacobian_file_ << q_(i) << "\t";
			}
			for (int i = 0; i < 3; i++)
			{
				simple_jacobian_file_ << x_(i) << "\t";
			}
			simple_jacobian_file_ << endl;
		}
		break;
	}
	case FEEDBACK_JACOBIAN:
	{
		Vector3d x_desired;
		double duration = 5.0;
		x_desired << 0.6, 0.2, 0.1;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		Matrix<double, 7, 3> j_v_inverse;
		j_v_inverse = j_v_.transpose() * (j_v_ * j_v_.transpose()).inverse();
		q_desired_ = q_ + j_v_inverse * (x_cubic_ - x_);
		if (play_time_ <= control_start_time_ + 7.0)
		{
			for (int i = 0; i < 7; i++)
			{
				feedback_jacobian_file_ << q_(i) << "\t";
			}
			for (int i = 0; i < 3; i++)
			{
				feedback_jacobian_file_ << x_(i) << "\t";
			}
			feedback_jacobian_file_ << endl;
		}
		break;
	}
	case CLIK:
	{
		Vector3d x_desired;
		Vector3d x_dot_desired;
		Matrix3d kp;
		kp = Matrix3d::Identity() * 1.0;

		double duration = 5.0;
		x_desired << 0.6, 0.2, 0.1;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		Matrix<double, 7, 3> j_v_inverse;
		x_dot_desired = (x_cubic_ - x_cubic_old_);
		x_cubic_old_ = x_cubic_;
		j_v_inverse = j_v_.transpose() * (j_v_ * j_v_.transpose()).inverse();
		q_desired_ = q_ + j_v_inverse * (x_dot_desired + kp * (x_cubic_ - x_));
		if (play_time_ <= control_start_time_ + 7.0)
		{
			for (int i = 0; i < 7; i++)
			{
				clik_file_ << q_(i) << "\t";
			}
			for (int i = 0; i < 3; i++)
			{
				clik_file_ << x_(i) << "\t";
			}
			clik_file_ << endl;
		}
		break;
	}
	case CLIK_WITH_WEIGHTED_PSEUDO_INV:
	{
		Vector3d x_desired;
		Vector3d x_dot_desired;
		Matrix7d w;
		Matrix3d kp;
		kp = Matrix3d::Identity() * 1.0;
		w = Matrix7d::Identity();
		w(3, 3) = 0.001;

		double duration = 5.0;
		x_desired << 0.6, 0.2, 0.1;
		x_cubic_ = DyrosMath::cubicVector<3>(play_time_,
			control_start_time_, control_start_time_ + duration,
			x_init_, x_desired, Vector3d::Zero(), Vector3d::Zero());
		Matrix<double, 7, 3> j_v_inverse;
		x_dot_desired = (x_cubic_ - x_cubic_old_);
		x_cubic_old_ = x_cubic_;
		j_v_inverse = w * j_v_.transpose() * (j_v_ * w * j_v_.transpose()).inverse();
		q_desired_ = q_ + j_v_inverse * (x_dot_desired + kp * (x_cubic_ - x_));
		if (play_time_ <= control_start_time_ + 7.0)
		{
			for (int i = 0; i < 7; i++)
			{
				clik_weight_file_ << q_(i) << "\t";
			}
			for (int i = 0; i < 3; i++)
			{
				clik_weight_file_ << x_(i) << "\t";
			}
			clik_weight_file_ << endl;
		}
		break;
	}
	default:
		break;
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}

void ArmController::setMode(ARM_CONTROL_MODE mode)
{
    is_mode_changed_ = true;
    control_mode_ = mode;
    cout << "Current mode (changed) : " ;

    switch (control_mode_)
    {

	// TODO: Implement this if you want
    case NONE:
        cout << "NONE";
        break;
	case HOME_JOINT_CTRL:
		cout << "HOME_JOINT_CTRL";
		break;
	case INIT_JOINT_CTRL:
		cout << "INIT_JOINT_CTRL";
		break;
	case SIMPLE_JACOBIAN:
		cout << "SIMPLE_JACOBIAN";
		break;
	case FEEDBACK_JACOBIAN:
		cout << "FEEDBACK_JACOBIAN";
		break;
	case CLIK:
		cout << "CLIK";
		break;
	case CLIK_WITH_WEIGHTED_PSEUDO_INV:
		cout << "CLIK_WITH_WEIGHTED_PSEUDO_INV";
		break;
    default:
        cout << "???";
        break;
    }
    cout << endl;
}


void ArmController::moveJointPosition(const Vector7d &target_pos, double duration)
{
	Vector7d zero_vector;
	zero_vector.setZero();
	q_desired_ = DyrosMath::cubicVector<7>(play_time_,
		control_start_time_,
		control_start_time_ + duration, q_init_, target_pos, zero_vector, zero_vector);
}


void ArmController::initFileDebug()
{
	simple_jacobian_file_.open("simple_jacobian.txt");
	feedback_jacobian_file_.open("feedback_jacobian.txt");
	clik_file_.open("clik.txt");
	clik_weight_file_.open("clik_weight.txt");
}


void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 2.)
	{
		DBG_CNT = 0;

		cout << "q now:\t";
		for (int i = 0; i<dof_; i++)
		{
			cout << std::fixed << std::setprecision(3) << q_(i) << '\t';
		}
		cout << endl;

		cout << "q desired:\t";
		for (int i = 0; i<dof_; i++)
		{
			cout << std::fixed << std::setprecision(3) << q_desired_(i) << '\t';
		}
		cout << endl;

		cout << "t:\t";
		for (int i = 0; i<dof_; i++)
		{
			cout << std::fixed << std::setprecision(3) << torque_(i) << '\t';
		}
		cout << endl;

		cout << " [taskState_ x] \n";
		cout << x_ << endl;
	}
}



// Controller Core Methods ----------------------------

void ArmController::initDimension()
{
    dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);

	x_target_.setZero();
	q_desired_.setZero();
}

void ArmController::initModel()
{
    model_ = make_shared<Model>();

    model_->gravity = Vector3d(0., 0, -GRAVITY);

    double mass[DOF];
    mass[0] = 1.0;
    mass[1] = 1.0;
    mass[2] = 1.0;
    mass[3] = 1.0;
    mass[4] = 1.0;
    mass[5] = 1.0;
    mass[6] = 1.0;

    Vector3d axis[DOF];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0*Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0*Eigen::Vector3d::UnitY();
	axis[6] = -1.0*Eigen::Vector3d::UnitZ();


	Eigen::Vector3d global_joint_position[DOF];

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

	for (int i = 0; i < DOF; i++)
		com_position_[i] -= global_joint_position[i];

    Math::Vector3d inertia[DOF];
	for (int i = 0; i < DOF; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

    for (int i = 0; i < DOF; i++) {
        body_[i] = Body(mass[i], com_position_[i], inertia[i]);
        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0)
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
        else
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    }
}

void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

void ArmController::writeData(Vector7d &position, Vector7d &velocity, Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		position(i) = q_desired_(i);
	}

}
void ArmController::writeData(Vector7d &position)
{
	position = q_desired_;
}



void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
}

// ----------------------------------------------------

