## AutonomousControl

This is my personal repo of control, planning and estimation algorithms for autonomous flight 

## Control Algorithms

1. PID Control 
2. [LQR_Control](#lqr) 
3. Geometric Control 
4. Adaptive Control 
5. Robust Control 

## <a name="lqr"></a>LQR_Control

## Compling and runing the algorithm

#### Dependencies 
Eigen 

#### Install 
Clone the repository 
```
git clone git@github.com:sandeshthapa/AutonomousControl.git
cd ~/AutonomousControl/State_Dep_LQR_Quad/
g++ -std=c++11 -o test State_Dependent_LQR.cpp 
./test
```

## Simulation 
1. Simulation in ROS/Gazebo 
2. PX4 SITL 
3. MAVROS Controllers

## Planning Algorithms 

1. Minimum Snap 

## Estimation 

1. EKF 

## Build issues:
fatal error: Eigen/Dense: No such file or directory
Add 
```
#include <eigen3/Eigen/Core>
```
instead of
```
#include <Eigen/Core>
```

Or compile with 
```
g++ -I /usr/local/include/eigen3 myfile.cpp -o filetest
```

### Contact 
thapasandesh1@gmail.com

