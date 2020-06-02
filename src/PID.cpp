#include "PID.h"
#include <math.h>

#include <cstdlib>
#include <iostream>
#include <cmath>

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
 
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  best_err=0;
  
  dp= {0.02, 0.001, 1};
  P_update=false;
  I_update=false;
  D_update=false;
  param=0;
  cout<<"called";

}

void PID::UpdateError(double cte) {
 
  double pre_cte = p_error;

  p_error  = cte;
  i_error += cte;
  d_error  = cte - pre_cte;
  
  

}

double PID::TotalError() {

  return -Kp*p_error - Ki*i_error - Kd*d_error;  
}

/* Initalize error */
void PID::Twiddle_init(double cte)
{
	 best_err=cte;
     std::vector<double> p = {this->Kp, this->Ki, this->Kd};
}

/*Stage 1 updates inital parameter and sees what happens */
void PID::Twiddle_stage1(double cte, double tolerance)
{
  double dparam_sum=dp[0]+dp[1]+dp[2];
  if(fabs(dp[param])>fabs(tolerance))
  {
    if(param==0)
   		Kp+=dp[param];
    else if(param==1)
    	Ki+=dp[param];
    else
  		Kd+=dp[param];  
  }
//   else
//     P_update= true;
 
}

/*Stage 2 Checks effect of update from stage 1 and updates params accordingly*/
bool PID::Twiddle_stage2(double cte,double tol)
{
   double dparam_sum=dp[0]+dp[1]+dp[2];
   if(fabs(dp[param])>fabs(tol))
   {
   		if(cte<best_err)
        {
        	best_err=cte;
            dp[param]*=1.1;
            return true;
        }
        else
        {
          if(param==0)
          {
           	Kp -= 2 * dp[param];
           	if(Kp<=0)
            	 Kp=0;
          }
          else if(param==1)
          {
            Ki -= 2 * dp[param];
           	if(Ki<=0)
            	 Ki=0;
            
          }
          else if(param==2)
          {
            Kd -= 2 * dp[param];
           	if(Kd<=0)
            	 Kd=0;
            
          }
             
           return false;
        }
   }
//   else
//     P_update= true;
 
}
/*checks the result of stage 2 and updates back to original in case of failure to improve error and reduces dp*/
/* if success improves the dp*/
void PID::Twiddle_stage3(double cte)
{
	 if (cte < best_err)
     {
  		best_err = cte;
        dp[param]*=1.1;
      
     }
     else
     {
       if(param==0)
         Kp += dp[param];
       else if(param==1)
          Ki += dp[param];
       else if(param==2)
          Kd += dp[param];
        dp[param] *= 0.9;
       
      }
}