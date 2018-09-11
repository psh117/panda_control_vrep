#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>


using namespace std;

extern "C" {
#include "extApi.h"
}

const std::string JOINT_HANDLE_PREFIX{ "panda_joint" };

class VRepBridge
{
private:
	typedef std::function<void()> callfunc; // loop callback function

public:
	VRepBridge();
	~VRepBridge();
	
	bool simConnectionCheck();
	void simLoop();

	void write();
	void read();


public:
	Eigen::Matrix<double, DOF, 1> current_q_;
	Eigen::Matrix<double, DOF, 1> current_q_dot_;
	Eigen::Matrix<double, DOF, 1> desired_q_;
	Eigen::Matrix<double, DOF, 1> desired_torque_;

	const size_t getTick() { return tick_; }

private:
	simxInt clientID_;
	simxInt motorHandle_[DOF];	/// < Depends on simulation envrionment
	simxInt objectHandle_;

	size_t tick_{ 0 };
	//callfunc loopCallbackFunc;

	void simxErrorCheck(simxInt error);
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
