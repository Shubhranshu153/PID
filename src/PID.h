#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);
  void Twiddle(double tolerance, double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  void Twiddle_init(double cte);
  void Twiddle_stage1(double cte, double tol);
  bool Twiddle_stage2(double cte,double tol);
  void Twiddle_stage3(double cte);
  void param_num();
  int param;
  bool P_update;
  bool D_update;
  bool I_update;
 

//  private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  double best_err;
  std::vector<double> dp;
};

#endif  // PID_H