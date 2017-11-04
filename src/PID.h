#ifndef PID_H
#define PID_H

#include <cmath>

class PID {
public:
  /*
  * Errors
  */
  double p_error=NAN;
  double i_error=NAN;
  double d_error=NAN;

  /*
  * Coefficients
  */ 
  double kP=NAN;
  double kI=NAN;
  double kD=NAN;

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
  const double UpdateError(const double theCrossTrackError, const double theSpeed);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
private:
  double previousCrossTrackError=NAN;
  double sumOfAllCrossTrackErrors=0.;
};

#endif /* PID_H */
