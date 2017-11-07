#ifndef PID_H
#define PID_H

#include <cmath>

class Sigmoid {
public:
  /*
   * Constructor
   */
  Sigmoid(){};
  
  /*
   * Destructor.
   */
  virtual ~Sigmoid(){};
  
  // pure virtual function providing interface framework.
  virtual const double getValue(const double x) const {
    return 0.;
  };
};

class SimpleSigmoid: public Sigmoid {
  
public:
  /*
   * Constructor
   */
  SimpleSigmoid(){};
  
  /*
   * Destructor.
   */
  ~SimpleSigmoid(){};
  
  const double getValue(const double x) const override {
    return 1./(1.+exp(-x));
  }
};

class SimpleSlopedSigmoid: public Sigmoid {
  
private:
  const double slope;
  
public:
  /*
   * Constructor
   */
  SimpleSlopedSigmoid(const double theSlope) : slope(theSlope) {};
  
  /*
   * Destructor.
   */
  ~SimpleSlopedSigmoid(){};
  
  const double getValue(const double x) const override {
    // (e^(.5*x) - 1.)/(1. + e^(.5*x))
    return 1./(1.+exp(-slope*x));
  }
};

class P1M1Sigmoid: public Sigmoid {
  
public:
  /*
   * Constructor
   */
  P1M1Sigmoid(){};
  
  /*
   * Destructor.
   */
  ~P1M1Sigmoid(){};
  
  const double getValue(const double x) const override {
    // (e^x - 1.)/(1. + e^x)
    return (exp(x)-1.)/(1.+exp(x));
  }
};

class P1M1SlopedSigmoid: public Sigmoid {

private:
  const double slope;
  
public:
  /*
   * Constructor
   */
  P1M1SlopedSigmoid(const double theSlope) : slope(theSlope) {};
  
  /*
   * Destructor.
   */
  ~P1M1SlopedSigmoid(){};
  
  const double getValue(const double x) const override {
    // (e^(.5*x) - 1.)/(1. + e^(.5*x))
    return (exp(slope*x)-1.)/(1.+exp(slope*x));
  }
};

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
  PID(Sigmoid &theSigmoid);

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
  virtual const double Update(const double theControlSignal);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
private:
  
  const bool DEBUGPRINT=false;
  
  double maximumControlSignal=0.;
  double maximumDifferentialControlSignal=0;
  double minimumControlSignal=0.;
  double minimumDifferentialControlSignal=0;
  
  const int differentialControlSignalHistorySize=3;
  int nextHistoryIndex=0;
  double *differentialControlSignalHistory=new double[differentialControlSignalHistorySize];
  

protected:
  
  double previousControlSignal=NAN;
  double sumOfAllControlSignals=0.;
  
  const Sigmoid &sigmoid;
  
  const void recordExtremes(const double theControlSignal, const double theDifferentialControlSignal);
  const void printExtremes();
  const double averageDifferentialControlSignal(const double theDifferentialControlSignal);

};

class ThrottlePID : public PID {
  
private:
  const double speedReference;
  const bool DEBUGPRINT=false;
  
public:
  /*
   * Constructor
   */
  ThrottlePID(Sigmoid &theSigmoid, const double theSoeeedReference);
  
  /*
   * Destructor.
   */
  virtual ~ThrottlePID(){};
  
  const double /* throttle */ Update(const double theControlSignal);
  
  
};

class CarControl {
  
private:
  
  PID* steeringPID;
  PID* throttlePID;

  const bool DEBUGPRINT=false;
  
public:

  /*
   * Constructor
   */
  CarControl();

  /*
  * Destructor.
  */
  virtual ~CarControl(){};

  /*
   * Update the CarControl.
   */
  const double* Update(const double theContro, const double theSpeed);
};

#endif /* PID_H */
