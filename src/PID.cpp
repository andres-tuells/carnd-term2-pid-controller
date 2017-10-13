#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.

def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    prev_cte = robot.y
    int_cte = 0
    for i in range(n):
        cte = robot.y
        diff_cte = cte - prev_cte
        prev_cte = cte
        int_cte += cte
        steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
    return x_trajectory, y_trajectory

*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	//initialize variables
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    //p_error = d_error = i_error = 0.0;

}

void PID::UpdateError(double cte) {
  // diff_cte=cte - prev_cte
  d_error = (cte - p_error);
  //new cte
  p_error = cte;
  //integral cte
  i_error += cte;
}




double PID::TotalError() {
	// steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
  	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

