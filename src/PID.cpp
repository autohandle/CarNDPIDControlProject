#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(Sigmoid &theSigmoid) : sigmoid(theSigmoid) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  kP=Kp;
  kI=Ki;
  kD=Kd;
}

const void PID::recordExtremes(const double theControlSignal, const double theDifferentialControlSignal) {
  maximumControlSignal=max(maximumControlSignal,theControlSignal);
  minimumControlSignal=min(minimumControlSignal,theControlSignal);

  maximumDifferentialControlSignal=max(maximumDifferentialControlSignal,theDifferentialControlSignal);
  minimumDifferentialControlSignal=min(maximumDifferentialControlSignal,theDifferentialControlSignal);
}

const void PID::printExtremes() {
  std::cout << "maxCTE:" << maximumControlSignal << ", minCTE:" << minimumControlSignal << std::endl
            << ", maxdCTE:" << maximumDifferentialControlSignal << ", mindCTE:" << minimumDifferentialControlSignal << std::endl;
}

const double PID::averageDifferentialControlSignal(const double theDifferentialControlSignal) {
  // current length
  if (nextHistoryIndex+1==differentialControlSignalHistorySize) {// is the history buffer full?
    for (int d=0; d<nextHistoryIndex; d++) {// shift history to the left to open up the last slot
      differentialControlSignalHistory[d]=differentialControlSignalHistory[d+1];
    }
  }
  differentialControlSignalHistory[nextHistoryIndex]=theDifferentialControlSignal;
  nextHistoryIndex=(nextHistoryIndex+1)%differentialControlSignalHistorySize;// should stop at: historySize-1
  double sumDifferentialControlSignal=0.;
  int numberOfEntries=min(nextHistoryIndex+1,differentialControlSignalHistorySize);
  for (int d=0; d<numberOfEntries; d++) {
    sumDifferentialControlSignal+=differentialControlSignalHistory[d];
  }
  return sumDifferentialControlSignal/((double)numberOfEntries);
}


//clamped: maximumControlSignal:3.0122, minimumControlSignal:-0.0199, maximumDifferentialControlSignal:0.426, minimumDifferentialControlSignal:-0.0381

const double /* new control value */ PID::Update(const double theControlSignal) {
  if (std::isnan(previousControlSignal)) {
    previousControlSignal=theControlSignal;
  }
  const double differentialControlSignal = theControlSignal-previousControlSignal;
  const double averageDCTE = averageDifferentialControlSignal(differentialControlSignal);
  const double newControl =  -kP * theControlSignal \
                          -kD *  differentialControlSignal \
                          -kI * sumOfAllControlSignals;
  previousControlSignal=theControlSignal;
  sumOfAllControlSignals+=theControlSignal;
  recordExtremes(theControlSignal, differentialControlSignal);
  if (DEBUGPRINT) printExtremes();
  const double clampedControl = min(max(newControl, -1.), 1.);
  if (DEBUGPRINT) std::cout << "PID-CTE:" << theControlSignal \
                            << ", dCTE:" << differentialControlSignal \
                            << ", sumCTE:" << sumOfAllControlSignals \
                            << ", avgdCTE:" << averageDCTE \
                            << std::endl;
  if (DEBUGPRINT) std::cout << "PID-newControl:" << newControl << ", clampedControl:" << clampedControl << std::endl;
  
  //return clampedControl;
  const double sigmoidControl = sigmoid.getValue(newControl);
  if (DEBUGPRINT) std::cout << "PID-newControl:" << newControl << ", sigmoidControl:" << sigmoidControl << std::endl;
  return sigmoidControl;

}

double PID::TotalError() {
}

ThrottlePID::ThrottlePID(Sigmoid &theSigmoid, const double theSpeedReference) : PID(theSigmoid), speedReference(theSpeedReference) {
}

const double /* throttle */ ThrottlePID::Update(const double theCrossTrackingError) {// throttle between 0 & 1
  // PID::Update between -1 & 1 -> throttleFactor is 0 at 1 & 1 at 0 -> between 0 & 1
  double pidControl=PID::Update(theCrossTrackingError);// -1 -> +1
  double throttleFactor=1.-abs(pidControl);// abs: +1 -> 0 -> +1 // 1-abs: 0 -> +1 -> 0
  if (DEBUGPRINT) std::cout << "ThrottlePID-pidControl:" << pidControl << ", throttleFactor:" << throttleFactor << std::endl;
  if (DEBUGPRINT) PID::printExtremes();
  double newSpeed=throttleFactor*speedReference;
  if (DEBUGPRINT) std::cout << "ThrottlePID-theCrossTrackingError:" << theCrossTrackingError << ", newSpeed:" << newSpeed << std::endl;
  return newSpeed;
  
}

// stable    throttle:0.4 maxCTE:4.6634, minCTE:-2.281 , maxdCTE:1.0997, mindCTE:-0.0002
// stable    throttle:0.8 Ks:1.,.001,50. Kt:0,0,100 maxCTE:4.7047,minCTE:-2.9494,maxdCTE:0.6007, mindCTE:0.0358
// stable    throttle:0.8 Ks:2.,.001,50. Kt:0,0,100 maxCTE:5.197,minCTE:-2.4486,maxdCTE:0.9299, mindCTE:-0.1169
// unstable  throttle:0.8 Ks:.5,.001,50. Kt:0,0,100 maxCTE:9.4153,minCTE:-3.985,maxdCTE:1.0062, mindCTE:-0.0637
// stable    throttle:0.8 Ks:1.,.001,25. Kt:0,0,100 maxCTE:3.3424,minCTE:-3.4471,maxdCTE:0.4412, mindCTE:0.0411
// unstable  throttle:0.8 Ks:1.,.001,15. Kt:0,0,100 maxCTE:6.0274,minCTE:-6.272,maxdCTE:1.4978, mindCTE:-0.0074
// stable    throttle:0.8 Ks:1.,.001,20. Kt:0,0,100 maxCTE:3.5335,minCTE:-3.7681,maxdCTE:1.7446, mindCTE:0.0561
// unstable  throttle:0.8 Ks:1.,.001,30. Kt:0,0,100 maxCTE:24.1299,minCTE:-3.637,maxdCTE:0.9075, mindCTE:-0.2219
// stableX   throttle:0.8 Ks:1.,.001,25. Kt:0,0,100 maxCTE:3.3424,minCTE:-3.4471,maxdCTE:0.4412, mindCTE:0.0411
// stable    throttle:0.8 Ks:.9,.001,25. Kt:0,0,100 maxCTE:3.4722,minCTE:-3.5238,maxdCTE:0.5117, mindCTE:-0.0364
// stable    throttle:0.8 Ks:.9,.001,25. Kt:1,0,100 maxCTE:3.8671,minCTE:-3.566, maxdCTE:1.5902, mindCTE:-0.0082
// stable    throttle:0.8 Ks:.9,.001,25. Kt:2,0,100 maxCTE:3.4327, minCTE:-3.8073, maxdCTE:2.099, mindCTE:0.0036
// stable    throttle:0.8 Ks:.9,.001,25. Kt:2,0,100 maxCTE:3.8947, minCTE:-3.7933, maxdCTE:1.279, mindCTE:-0.0138
// stable    throttle:0.8 Ks:.9,.001,25. Kt:2,0,100 maxCTE:3.4584, minCTE:-3.9159, maxdCTE:0.8101, mindCTE:-0.0664

CarControl::CarControl() :  steeringPID(new PID(*(new P1M1SlopedSigmoid(0.207)))),
throttlePID(new ThrottlePID(*(new P1M1SlopedSigmoid(.095)/* throttle between 0 & 1*/), 1. /* speed reference */)) {
  (*steeringPID).Init(1.1/* Kp */, .00001 /* Ki */, 21. /* Kd */);//
  (*throttlePID).Init(10./* Kp */, 0. /* Ki */, 100. /* Kd */);//
}

/*
 * Update the PID error variables given cross track error.
 */
const double* CarControl::Update(const double theCTE, const double theSpeed) {// double[0]: steering, double[1]: throttle
  double steering=(*steeringPID).Update(theCTE);
  double speed=(*throttlePID).Update(theCTE);
  static double controls[2];
  controls[0]=steering;
  controls[1]=speed;
  if (DEBUGPRINT) std::cout << "CarControl-steering:" << controls[0] << ", speed:" << controls[1] << std::endl;
  return controls;
}
/*
 def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
 x_trajectory = []
 y_trajectory = []
 # TODO: your code here
 deltaT=1.0
 previousControlSignalError=robot.y
 sumOfAllControlSignalErrors=0.
 for t in range(n):
 ControlSignalError = robot.y
 sumOfAllControlSignalErrors+=ControlSignalError
 differentialControlSignalError = ControlSignalError-previousControlSignalError
 steering=steering = -tau_p * ControlSignalError \
 -tau_d * differentialCrossTrackError \
 -tau_i * sumOfAllCrossTrackErrors
 distance=deltaT*speed
 """
 steering = front wheel steering angle, limited by max_steering_angle
 distance = total distance driven, most be non-negative
 """
 robot.move(steering, distance)
 x_trajectory.append(robot.x)
 y_trajectory.append(robot.y)
 previousCrossTrackError=crossTrackError
 #print ("t:"+repr(t)+", x:"+repr(robot.x)+", y:"+repr(robot.y)+", orientation:"
 #    +repr(robot.orientation)+", steering:"+repr(steering))
 return x_trajectory, y_trajectory
 */
