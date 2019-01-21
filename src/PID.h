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

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
    
    //choose the parameter to modefy
    void ChooseTheParameter(int index, double dp_para);

 private:
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
    
    //add some value
    double pre_cte;
    double total_error;
    
    //add twidodle value
    int step;
    bool use_twiddle;
    int front_loop, back_loop;
    double sum_err, best_err;
    
    int modify_index;
    std::vector<double> dp;
    
    bool flag1, flag2;
};

#endif  // PID_H
