#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double last_cte;

  /*
  * Coefficients
  */
  double Kp_;
  double Ki_;
  double Kd_;

  double dp_;
  double di_;
  double dd_;

  double best_err_;
  double total_error_;

  int Kp_changed_;
  int Kp_changed2_;
  int Kp_changed3_;

  int Ki_changed_;
  int Ki_changed2_;
  int Ki_changed3_;

  int Kd_changed2_;
  int Kd_changed3_;

  int time_steps_;
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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * get feasible steer value
  */
  double GetSteerValue();

  /*
  * Define twiddle to optimize PID parameters
  */
  void Twiddle();

  /*
  * reset assistant parameters in TWIDDLE
  */
  void Reset();
};

#endif /* PID_H */
