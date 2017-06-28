#include "PID.h"

using namespace std;

PID::PID() {
	p_error = i_error = d_error = 0.0;
	Kp = Ki = Kd = 0.0;
	is_initialized = false;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte) {
	if (is_initialized) {
		d_error = cte - p_error;
	} else {
		d_error = 0.0;
		is_initialized = true;
	}
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	/* p_error stores the previous cte */
	return p_error * p_error;
}

