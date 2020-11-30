#ifndef STATE_DEPENDENT_lQR
#define STATE_DEPENDENT_lQR

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
// #include <assert.h>
// #include <complex>
#include <iostream>
// #include <chrono>
// #include <stdio.h>
// #include <bits/stdc++.h>
// #include <vector>
// #include <cmath>

using namespace Eigen;
using namespace std; 

class State_Dependent_LQR
{
private:
    double mass;
    double arm_len; 
    double I_xx; 
    double I_yy; 
    double I_zz; 
public:

   

    // Constructor
    State_Dependent_LQR(double mass_, double arm_len_);

    // Destrcutor
    ~State_Dependent_LQR();

    void setinertialparams(double mass_, double arm_len_); 

    struct care_solver; 
     double gravity; 
    // double mass = 2.8; //0.865 ; // [kg]
    // double I_xx = 0.0551;
    // double I_yy = 0.0551; //
    // double I_zz = 0.0858; //

    care_solver care_soln(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q,const MatrixXd &R );

    Eigen::Matrix<double, 12, 12> A_LQR(const Eigen::Matrix<double, 13, 1> & state_estimate, const Eigen::Matrix<double, 4, 1> & u);

    Eigen::Matrix<double, 12, 4> B_LQR( const Eigen::Matrix<double, 13, 1> & state_estimate,  const Eigen::Matrix<double, 4, 1> & u);

    Eigen::Matrix<double, 4,1> compute_Ulqr_12_states( const Eigen::Matrix<double, 13, 1> &reference_state,
                                                                            const Eigen::Matrix<double, 13, 1> & state_estimate );

  Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q);

    
};

struct State_Dependent_LQR::care_solver
    {
        Eigen::MatrixXd X; // Solution to the care
        Eigen::MatrixXd K;
    };


#endif 


