#ifndef PID_H
#define PID_H

/*
 * This class is only used for storing the internal
 * state of the PID controller.
 */
class PID {

private:
	bool is_initialized;
	unsigned int nSteps;
	unsigned int nStepsCumulativeErrorAccumulated;
	double cumulative_error;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, bool computeCumulativeError);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double CurrentError();

  /*
   * Compute the PID control value.
   */
  double ComputeControlValue();

  /*
   * Gets number of control steps.
   */
  int GetNumSteps();

  /*
   * Prints internal state.
   */
  void PrintInternalState();

  /*
   * Resets internal state.
   */
  void ResetInternalState(double Kp, double Ki, double Kd);
};

#endif /* PID_H */
