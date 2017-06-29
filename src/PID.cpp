#include<iostream>
#include "PID.h"

using namespace std;

PID::PID() {
	p_error = i_error = d_error = 0.0;
	is_initialized = false;
	nSteps = 0;
	cumulative_error = 0.0;
	Kp = Ki = Kd = 0.0;
	nStepsCumulativeErrorAccumulated = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::ResetInternalState(double _Kp, double _Ki, double _Kd) {
	p_error = i_error = d_error = 0.0;
	is_initialized = false;
	nSteps = 0;
	cumulative_error = 0.0;
	Init(_Kp, _Ki, _Kd);
}

void PID::UpdateError(double cte, bool computeCumulativeError) {
	if (is_initialized) {
		d_error = cte - p_error;
	} else {
		d_error = 0.0;
		is_initialized = true;
	}
	p_error = cte;
	i_error += cte;
	nSteps++;
	if (computeCumulativeError) {
		cumulative_error += (p_error * p_error);
		nStepsCumulativeErrorAccumulated++;
	}
}

double PID::CurrentError() {
	return (p_error * p_error);
}

double PID::TotalError() {
	return cumulative_error/nStepsCumulativeErrorAccumulated;
}

double PID::ComputeControlValue() {
	double value = - Kp * p_error - Kd * d_error - Ki * i_error;
	value = (value < -1.0)? -1.0: value;
	value = (value > 1.0)? 1.0: value;
	return value;
}

int PID::GetNumSteps() { return nSteps; }

void PID::PrintInternalState() {
	cout << "   PIDC Coeffs> " << "Kp = " << Kp << ", Kd = " << Kd << ", Ki = " << Ki << endl;
}
