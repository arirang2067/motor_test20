#include <cmath>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef struct pid_param
{
  double kP=0;
  double kI=0;
  double kD=0;
  double Imax=0;
  double Dmax=10;
} pid_param;

typedef struct pid
{
  double p_out=0;
  double integrator=0;
  double derivative=0;
  double last_input=0;
  double lastderivative=0;

  double output=0;
} pid;

double PidContoller(double goal, double error, double dt, pid *pid_data, pid_param *pid_paramdata, int error_rat)
{
  // ROS_INFO(" goal : %f, curr: %f, dt: %f", goal,curr,dt);
  // double error = goal - curr;
  // ROS_INFO(" error : %f", error);
  if (fabs(error) < error_rat)
    error = 0;

  pid_data->p_out = pid_paramdata->kP * error;
  double p_data = pid_data->p_out ;

  pid_data->integrator += (error * pid_paramdata->kI) * dt;
  pid_data->integrator = constrain(pid_data->integrator, -pid_paramdata->Imax, pid_paramdata->Imax);
  double i_data = pid_data->integrator;

  double filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  pid_data->derivative = (goal - pid_data->last_input) / dt;
  pid_data->derivative = pid_data->lastderivative + (dt / (filter + dt)) * (pid_data->derivative - pid_data->lastderivative);
  pid_data->last_input = goal;
  pid_data->lastderivative = pid_data->derivative;
  double d_data = pid_paramdata->kD * pid_data->derivative;
  d_data = constrain(d_data, -pid_paramdata->Dmax, pid_paramdata->Dmax);

  double output = p_data + i_data + d_data;
  pid_data->output = output;

  return pid_data->output;
}
