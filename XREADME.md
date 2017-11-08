# PID Controller

#### Compiling
##### Code must compile without errors with cmake and make.

``` shell
Softwares-MacBook-Pro:tmp david$ git clone https://github.com/autohandle/CarNDPIDControlProject
Cloning into 'CarNDPIDControlProject'...
remote: Counting objects: 89, done.
remote: Compressing objects: 100% (41/41), done.
remote: Total 89 (delta 46), reused 89 (delta 46), pack-reused 0
Unpacking objects: 100% (89/89), done.
Softwares-MacBook-Pro:tmp david$ cd CarNDPIDControlProject/
Softwares-MacBook-Pro:CarNDPIDControlProject david$ mkdir build
Softwares-MacBook-Pro:CarNDPIDControlProject david$ cd build
Softwares-MacBook-Pro:build david$ cmake ..
-- The C compiler identification is AppleClang 9.0.0.9000037
-- The CXX compiler identification is AppleClang 9.0.0.9000037
-- Check for working C compiler: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc
-- Check for working C compiler: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++
-- Check for working CXX compiler: /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: /tmp/CarNDPIDControlProject/build
Softwares-MacBook-Pro:build david$ make
Scanning dependencies of target pid
[ 33%] Building CXX object CMakeFiles/pid.dir/src/PID.cpp.o
/tmp/CarNDPIDControlProject/src/PID.cpp:83:1: warning: control reaches end of non-void function [-Wreturn-type]
}
^
1 warning generated.
[ 66%] Building CXX object CMakeFiles/pid.dir/src/main.cpp.o
/tmp/CarNDPIDControlProject/src/main.cpp:58:18: warning: unused variable 'angle' [-Wunused-variable]
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                 ^
1 warning generated.
[100%] Linking CXX executable pid
ld: warning: directory not found for option '-L/usr/local/Cellar/libuv/1.11.0/lib'
[100%] Built target pid
Softwares-MacBook-Pro:build david$ ./pid
Listening to port 4567
^C
Softwares-MacBook-Pro:build david$
```

#### Implementation
##### The PID procedure follows what was taught in the lessons

The algorithm is implemented in
[PID.cpp](https://github.com/autohandle/CarNDPIDControlProject/blob/master/src/PID.cpp) and [PID.h](https://github.com/autohandle/CarNDPIDControlProject/blob/master/src/PID.h) in the class [CarControl](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.cpp#L115-L132). [CarControl](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.cpp#L115-L132) is both initialized
``` C++
CarControl carControl;
```
and called by [main.cpp](https://github.com/autohandle/CarNDPIDControlProject/blob/master/src/main.cpp) in the event [on_message](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/main.cpp#L42-L88).
``` C++
const double* carControls=carControl.Update(cte, speed);
steer_value=*carControls;
throttle=carControls[1];
// DEBUG
std::cout << "CTE: " << cte << " Steering Value: " << steer_value << ", throttle:" << throttle << std::endl;
```
[CarControl](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.cpp#L115-L132) returns an updated control value for both the `steer_value` and the `throttle` value to be sent to the simulator.

[CarControl::Update](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.cpp#L124-L132) 
delegates the steering update to one PID controller and the throttle/speed update to another,
``` C++
double steering=(*steeringPID).Update(theCTE);
double speed=(*throttlePID).Update(theCTE);
static double controls[2];
controls[0]=steering;
controls[1]=speed;
if (DEBUGPRINT) std::cout << "CarControl-steering:" << controls[0] << ", speed:" << controls[1] << std::endl;
return controls;
```
and then returns the control updates to `main`. The PID controllers were initialized in the [CarControl::CarControl](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.cpp#L115-L119) constructor.
``` C++
CarControl::CarControl() :  steeringPID(new PID(*(new P1M1SlopedSigmoid(0.207)))),
throttlePID(new ThrottlePID(*(new SimpleSlopedSigmoid(5.)/* throttle between 0 & 1*/), 0.7 /* speed reference */)) {
  (*steeringPID).Init(1.1/* Kp */, .00001 /* Ki */, 21. /* Kd */);//
  (*throttlePID).Init(10./* Kp */, 0. /* Ki */, 100. /* Kd */);//
}
```
The streering PID update, [PID::Update](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.cpp#L54-L80), implements the 3 parameter controller discussed in class:
``` C++
const double newControl =  -kP * theControlSignal \
                        -kD *  differentialControlSignal \
                        -kI * sumOfAllControlSignals;

...

const double sigmoidControl = sigmoid.getValue(newControl);
```
The `newControl` value is clamped by using a [sigmoid](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.h#L24-L40)
<br>
![sigmoid](./images/LogisticFunction.png)
<br>
whose range has been [adjusted](https://github.com/autohandle/CarNDPIDControlProject/blob/590473f6dcaee40b02275dbb96be24f5be064f0f/src/PID.h#L64-L81) between -1 and 1
<br>
![sigmoid](./images/ErrorFunction.png)

#### Reflection
##### The effect each of the P, I, D components

The simulation shows a completion time of: 64 seconds. I ran the simulation several times and the maximum finishing time was 95 seconds — that worst-case performance can be viewed in the [video](https://s3.amazonaws.com/autohandle.com/video/CarNDKidnappedVehicleProject.mp4).
l
##### Describe how the final hyperparameters were chosen.

###### [ParticleFilter::init](https://github.com/autohandle/CarNDKidnappedVehicleProject/blob/24502292382ccf2178b8a5f79b45967ffa671ea8/src/particle_filter.cpp#L23-L49)

The main program first initializes the [ParticleFilter](https://github.com/autohandle/CarNDKidnappedVehicleProject/blob/master/src/particle_filter.cpp) in [ParticleFilter::init](https://github.com/autohandle/CarNDKidnappedVehicleProject/blob/24502292382ccf2178b8a5f79b45967ffa671ea8/src/particle_filter.cpp#L23-L49) with 500 particles in a random state (x,y, and theta) around an initial x,y gps reading,

``` C++
for (int particle = 0; particle < num_particles; ++particle) {
    double sample_x, sample_y, sample_theta;
    
    // TODO: Sample  and from these normal distrubtions like this:
    // sample_x = dist_x(gen);
    // where "gen" is the random engine initialized earlier.
    sample_x=dist_x(gen);
    sample_y=dist_y(gen);
    sample_theta=dist_theta(gen);
    addParticleToFilter(createParticle(sample_x, sample_y, sample_theta));
  }

  is_initialized=true;
```
then the `is_initialized` flag is set so that the main program does not try to initialize [ParticleFilter](https://github.com/autohandle/CarNDKidnappedVehicleProject/blob/master/src/particle_filter.cpp) again.

After initialization, the main program loops through: [ParticleFilter::prediction](https://github.com/autohandle/CarNDKidnappedVehicleProject/blob/24502292382ccf2178b8a5f79b45967ffa671ea8/src/particle_filter.cpp#L58-L98), [ParticleFilter::updateWeights](https://github.com/autohandle/CarNDKidnappedVehicleProject/blob/24502292382ccf2178b8a5f79b45967ffa671ea8/src/particle_filter.cpp#L145-L183), and [ParticleFilter::resample](https://github.com/autohandle/CarNDKidnappedVehicleProject/blob/24502292382ccf2178b8a5f79b45967ffa671ea8/src/particle_filter.cpp#L185-L240).

#### Simulation

[PID Controller Project](https://s3.amazonaws.com/autohandle.com/video/CarNDPIDControlProject70.mp4)

The video was created by using a [screen recording tool](http://www.idownloadblog.com/2016/02/26/how-to-record-part-of-mac-screen-quicktime/).

