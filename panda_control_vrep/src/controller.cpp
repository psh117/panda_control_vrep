#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>



void ArmController::compute()
{
	// Kinematics calculation ------------------------------
	q_temp_ = q_;
	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, NULL, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_pos_[DOF - 1], true);
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_pos_[DOF - 1], j_temp_, true);
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
		target_position << 0.0, 0.0, 0.0, M_PI/2., 0.0, 0.0, 0.0;
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
    mass[0] = 6.98;
    mass[1] = 1.0;
    mass[2] = 5.23;
    mass[3] = 6.99;
    mass[4] = 3.3;
    mass[5] = 2.59;
    mass[6] = 1.0;

    Vector3d axis[DOF];
    axis[0] = Vector3d(0.0, 0.0, -1.0);
    axis[1] = Vector3d(0, -1, 0);
    axis[2] = Vector3d(0, 0, -1);
    axis[3] = Vector3d(0, 1, 0);
    axis[4] = Vector3d(0, 0, -1);
    axis[5] = Vector3d(0, -1, 0);
    axis[6] = Vector3d(0, 0, -1);


    Math::Vector3d inertia[DOF];
    inertia[0] = Vector3d(0.02854672, 0.02411810, 0.01684034);
    inertia[1] = Vector3d(0.00262692, 0.00281948, 0.00214297);
    inertia[2] = Vector3d(0.04197161, 0.00856546, 0.04186745);
    inertia[3] = Vector3d(0.04906429, 0.03081099, 0.02803779);
    inertia[4] = Vector3d(0.00935279, 0.00485657, 0.00838836);
    inertia[5] = Vector3d(0.00684717, 0.00659219, 0.00323356);
    inertia[6] = Vector3d(0.00200000, 0.00200000, 0.00200000);

    joint_pos_[0] = Vector3d(0.0, 0.0, 0.0);
    joint_pos_[1] = Vector3d(0.0, 0.0, 0.223) - joint_pos_[0];
    joint_pos_[2] = Vector3d(0.0, 0.118, 0.223) - joint_pos_[1] - joint_pos_[0];
    joint_pos_[3] = Vector3d(0.0, 0.118, 0.5634) - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    joint_pos_[4] = Vector3d(0.0, 0.0, 0.5634) - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    joint_pos_[5] = Vector3d(0.0, 0.0, 0.8634) - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    joint_pos_[6] = Vector3d(0.0, 0.112, 0.8634) - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];

    com_pos_[0] = Vector3d(-0.00006, 0.04592, 0.20411) - joint_pos_[0];
    com_pos_[1] = Vector3d(0.00007, 0.12869, 0.24008) - joint_pos_[1] - joint_pos_[0];
    com_pos_[2] = Vector3d(0.00043, 0.1205, 0.38421) - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[3] = Vector3d(-0.00008, 0.05518, 0.59866) - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[4] = Vector3d(0.0, 0.01606, 0.74571) - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[5] = Vector3d(0.00002, 0.11355, 0.91399) - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];
    com_pos_[6] = Vector3d(-0.0, 0.112, 0.9246) - joint_pos_[6] - joint_pos_[5] - joint_pos_[4] - joint_pos_[3] - joint_pos_[2] - joint_pos_[1] - joint_pos_[0];

    for (int i = 0; i < DOF; i++) {
        body_[i] = Body(mass[i], com_pos_[i], inertia[i]);
        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0)
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_pos_[i]), joint_[i], body_[i]);
        else
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_pos_[i]), joint_[i], body_[i]);
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

