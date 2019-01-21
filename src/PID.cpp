#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    
    p_error = 0;
    d_error = 0;
    i_error = 0;
    
    //set twiddle value
    step = 1;
    use_twiddle = false;
    front_loop = 100;
    back_loop = 2000;
    sum_err = 0;
    best_err = std::numeric_limits<double>::max();
    dp = {Kp_*0.1, Ki_*0.1, Kd_*0.1};
    modify_index = 2;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
    if (step == 1) {
        pre_cte = cte;
    }
    p_error = cte;
    
    d_error = cte - pre_cte;
    pre_cte = cte;
    
    i_error += cte;
    
    //twiddle algorithm
    
    if (step%(front_loop + back_loop) > back_loop) {
        sum_err += pow(cte,2);
    }
    
    if (use_twiddle && step%(front_loop + back_loop) == 0) {
//        sum_err /= back_loop;

        if (sum_err < best_err) {
            cout << "have improment here." << endl;
            best_err = sum_err;
            //remove the fist in
            if (step%(front_loop + back_loop) != 0) {
                dp[modify_index] = dp[modify_index] * 1.1;
            }
            modify_index = (modify_index + 1)%3;
            flag1 = flag2 = false;
        }
        if (!flag1 && !flag2) {
            ChooseTheParameter(modify_index, dp[modify_index]);
            flag1 = true;
        }
        else if (flag1 && !flag2) {
            ChooseTheParameter(modify_index, -2*dp[modify_index]);
            flag2 = true;
        }
        else {
            ChooseTheParameter(modify_index, dp[modify_index]);
            dp[modify_index] = dp[modify_index] * 0.9;

            modify_index = (modify_index + 1)%3;
            flag1 = flag2 = false;
        }
        sum_err = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
    }
    
    
//    // update total error only if we're past number of settle steps
//    if (step % (front_loop + back_loop) > front_loop){
//        total_error += pow(cte,2);
//    }
//
//    // last step in twiddle loop... twiddle it?
//    if (use_twiddle && step % (front_loop + back_loop) == 0){
//        cout << "step: " << step << endl;
//        cout << "total error: " << total_error << endl;
//        cout << "best error: " << best_err << endl;
//        if (total_error < best_err) {
//            cout << "improvement!" << endl;
//            best_err = total_error;
//            if (step !=  front_loop + back_loop) {
//                // don't do this if it's the first time through
//                dp[modify_index] *= 1.1;
//            }
//            // next parameter
//            modify_index = (modify_index + 1) % 3;
//            flag1 = flag2 = false;
//        }
//        if (!flag1 && !flag2) {
//            // try adding dp[i] to params[i]
//            ChooseTheParameter(modify_index, dp[modify_index]);
//            flag1 = true;
//        }
//        else if (flag1 && !flag2) {
//            // try subtracting dp[i] from params[i]
//            ChooseTheParameter(modify_index, -2 * dp[modify_index]);
//            flag2 = true;
//        }
//        else {
//            // set it back, reduce dp[i], move on to next parameter
//            ChooseTheParameter(modify_index, dp[modify_index]);
//            dp[modify_index] *= 0.9;
//            // next parameter
//            modify_index = (modify_index + 1) % 3;
//            flag1 = flag2 = false;
//        }
//        total_error = 0;
//    }
    
    cout << "modify_index:"<< modify_index << endl;
    cout << "dp: " << dp[0] << "," << dp[1] << "," << dp[2] << "," << endl;
    cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
    cout << "step:"<< step << endl;
    step++;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    
    total_error = -Kp*p_error -Ki*i_error -Kd*d_error;
    
  return total_error;  // TODO: Add your total error calc here!
}

void PID::ChooseTheParameter(int index, double dp_para) {
    switch (index) {
        case 0:
            Kp += dp_para;
            break;
        case 1:
            Ki += dp_para;
            break;
        case 2:
            Kd += dp_para;
            break;
        
        default:
            break;
    }
}

