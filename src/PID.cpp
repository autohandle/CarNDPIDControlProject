#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  kP=Kp;
  kI=Ki;
  kD=Kd;
}

const double /* steering*/ PID::UpdateError(const double theCrossTrackError, const double theSpeed) {
  if (std::isnan(previousCrossTrackError)) {
    previousCrossTrackError=theCrossTrackError;
  }
  const double differentialCrossTrackError = theCrossTrackError-previousCrossTrackError;
  const double steering=  -kP * theCrossTrackError \
                          -kD * differentialCrossTrackError \
                          -kI * sumOfAllCrossTrackErrors;
  previousCrossTrackError=theCrossTrackError;
  sumOfAllCrossTrackErrors+=theCrossTrackError;
  const double clampedSteering = min(max(steering, -1.), 1.);
  //std::cout << "theCrossTrackError:" << theCrossTrackError << ", differentialCrossTrackError:" << differentialCrossTrackError << ", sumOfAllCrossTrackErrors:" << sumOfAllCrossTrackErrors << std::endl;
  std::cout << "steering:" << steering << ", clampedSteering:" << clampedSteering << std::endl;
  return clampedSteering;
}

double PID::TotalError() {
}

/*
 def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
 x_trajectory = []
 y_trajectory = []
 # TODO: your code here
 deltaT=1.0
 previousCrossTrackError=robot.y
 sumOfAllCrossTrackErrors=0.
 for t in range(n):
 crossTrackError = robot.y
 sumOfAllCrossTrackErrors+=crossTrackError
 differentialCrossTrackError = crossTrackError-previousCrossTrackError
 steering=steering = -tau_p * crossTrackError \
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
