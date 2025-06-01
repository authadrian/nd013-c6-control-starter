/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/

   k_p = Kpi;    // Kpi: Proportional gain - controls response in proportion to the current error
   k_d = Kdi;    // Kdi: Derivative gain - controls response based on the rate of change of error
   k_i = Kii;    // Kii: Integral gain - controls response based on the accumulated error over time

   output_limit_max = output_lim_maxi;    // Upper boundary for controller output
   output_limit_min = output_lim_mini;    // Lower boundary for controller output

   // Initialize all errors to zero
   error_p = 0.0;       // Current error (proportional term)
   error_d = 0.0;       // Error rate of change (derivative term)
   error_i = 0.0;       // Accumulated error (integral term)

   dt=0.0;              // Delta time between updates (initialized to zero)
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

  // Calculate derivative error (rate of change of error)
  // Only update if dt is valid to avoid division by zero
  if(dt>0){
      // Derivative error = change in error / change in time
      error_d = (cte - error_p) / dt;
  } else {
      // If no time has passed, rate of change is zero
      error_d = 0;
  }

  error_p = cte;           // Update the current error
  error_i += cte * dt;     // Update the accumulated error
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;

    // calulate control output of PID controller
    control = k_p * error_p + k_d * error_d + k_i * error_i;
   
    // Limit control output to be within boundary limits
    control = max(min(output_limit_max, control), output_limit_min);

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   dt = new_delta_time;
   return dt;
}