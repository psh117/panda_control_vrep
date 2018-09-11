#include <iostream>
#include <string>
#include <conio.h>
#include "vrep_bridge.h"

#include "controller.h"

using namespace std;



int main()
{
	VRepBridge vb;
	const double hz = 100;
	ArmController ac(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		vb.read();
		ac.readData(vb.current_q_, vb.current_q_dot_);
		if (is_first)
		{
			cout << vb.current_q_ << endl;
			is_first = false;
			//ac.initPosition();
		}

		if (_kbhit())
		{
			int key = _getch();

			switch (key)
			{
				// Implement with user input

			case 'i':
				cout << "Joint control to initial position" << endl;
				ac.setMode(ArmController::INIT_JOINT_CTRL);
				break;

			case 'h':
				cout << "Joint control to home position" << endl;
				ac.setMode(ArmController::HOME_JOINT_CTRL);
				break;

			case 's':
				//cout << "Joint control to home position" << endl;
				ac.setMode(ArmController::SIMPLE_JACOBIAN);
				break;
			case 'f':
				//cout << "Joint control to home position" << endl;
				ac.setMode(ArmController::FEEDBACK_JACOBIAN);
				break;
			case 'c':
				//cout << "Joint control to home position" << endl;
				ac.setMode(ArmController::CLIK);
				break;
			case 'w':
				//cout << "Joint control to home position" << endl;
				ac.setMode(ArmController::CLIK_WITH_WEIGHTED_PSEUDO_INV);
				break;

			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
			ac.writeData(vb.desired_q_);
		
			vb.write();
			vb.simLoop();
		}
	}
		
	return 0;
}
