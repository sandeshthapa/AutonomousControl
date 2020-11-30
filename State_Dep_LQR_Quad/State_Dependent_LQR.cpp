// Nov 2020. Sandesh Thapa, thapasandesh1@gmail.com

#include "State_Dependent_LQR.h"

State_Dependent_LQR::State_Dependent_LQR(double mass_, double arm_len_)
      : mass(mass_), 
        arm_len (arm_len_), 
        I_xx (0.0),
        I_yy  (0.0), 
        I_zz (0.0), 
        gravity (9.81)
   {} 

State_Dependent_LQR::~State_Dependent_LQR() 
    {
    }

/**
 * This functions computesia the inertia matrix for a given mass and arm lenght, for details view: 
 * Reference: 
 * 1. Flightmare: A Flexible Quadrotor Simulator, 
 *    Song, Yunlong and Naji, Selim and Kaufmann, Elia and Loquercio, Antonio and Scaramuzza, Davide
 * @param mass 
 * @param arm_length
 * @return none 
 * @author Sandesh Thapa
 */
void State_Dependent_LQR::setinertialparams(double mass_, double arm_len_)
{
    I_xx = mass_/12*arm_len_*arm_len_*4.5; 

    std::cout << "Ixx" << I_xx << std::endl; 

    I_yy = mass_/12*arm_len_*arm_len_*4.5; 

    I_zz = mass_/12*arm_len_*arm_len_*7; 

} 

/**
 *  This functions returns the solution of continuous time algebraic recatti equation
 * @param A; 
 * @param B; 
 * @param Q; 
 * @param R; 
 * @return Soln.K , Soln.X; // K is the controller gain and X is the solution to CARE
 * @author Sandesh Thapa, thapasandesh1@gmail.com
 */ 
State_Dependent_LQR::care_solver  State_Dependent_LQR::care_soln(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q,const MatrixXd &R )
{
    //assert(A.rows() == A.cols() && A.rows() == B.rows() && A.rows() == Q.rows() && A.rows() == Q.cols() && B.cols() == R.rows() && B.cols() == R.cols() && "CARE Solver: Check input matrix sizes");

    int N = A.rows();
    int M = B.cols();

    MatrixXd Rinv =  R.inverse(); // Varaible matrix of dynamic size

    MatrixXd Z; // The Hamiltonian matrix

    Z.resize(2*N,2*N); // Make Z size (24,24)

    // Z = [A+B*Rinv*B'*Ainv'*Q -B*Rinv*B'*Ainv'; -Ainv'*Q Ainv']; https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
    Z.block(0,0,N,N) = A;
    Z.block(0,N,N,N) = -B*Rinv*B.transpose();
    Z.block(N,0,N,N) = -Q;
    Z.block(N,N,N,N) = -A.transpose();

    // Finded the eigen vectors and eigen values matrix--> [V,L] = eig(Z)
    Eigen::EigenSolver<MatrixXd> eigZ(Z);
    //eigZ.compute(Z,true);

    Eigen::MatrixXcd U1;
    Eigen::MatrixXcd U2;
    U1.resize(N,N);
    U2.resize(N,N);

    Eigen::MatrixXcd Eigval   = eigZ.eigenvalues();
    Eigen::MatrixXcd Eigvec   =  eigZ.eigenvectors();

    //Assign U1 and U2 with U1 consisting only negative real parts
    int u_col = 0;
    for(int eigInd=0; eigInd < 2*N && u_col<N; eigInd++ )
    {
        if( Eigval(eigInd).real() < 0 )
        {
            U1.block(0,u_col,N,1) = Eigvec.block(0,eigInd,N,1);
            U2.block(0,u_col,N,1) = Eigvec.block(N,eigInd,N,1);
            u_col ++;
        }
    }

    care_solver lqr_soln;
    //Eigen::MatrixXcd U1 = U.block(0,0,N,N);
    Eigen::MatrixXcd X_c = U2*U1.inverse(); //Solution
    lqr_soln.X.resize(N,N);

    for(int i=0; i<N;i++)
        for(int j=0;j<N;j++)
            lqr_soln.X(i,j) = X_c(i,j).real();

    //Compute the gains
    lqr_soln.K = Rinv*B.transpose()*lqr_soln.X;

    // cout << "Solution for CARE: X" << lqr_soln.X << endl;
    // cout << "Gain K: "<< lqr_soln.K << endl;
    return lqr_soln;

}

/**
 *  This functions returns the A matrix for quadrotor UAV for details view the following references: 
 * 1. Bouabdallah, S., Noth, A., & Siegwart, R. (2004, September).
 *    PID vs LQ control techniques applied to an indoor micro quadrotor.
 *    In 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems
 *    (IROS)(IEEE Cat. No. 04CH37566) (Vol. 3, pp. 2451-2456). IEEE.
 * 2. Bouabdallah, S., & Siegwart, R. (2007, October). Full control of a quadrotor.
 *     In 2007 IEEE/RSJ International Conference on Intelligent Robots and Systems (pp. 153-158). Ieee.
 * 3. Flightmare: A Flexible Quadrotor Simulator, 
 *    Song, Yunlong and Naji, Selim and Kaufmann, Elia and Loquercio, Antonio and Scaramuzza, Davide
 * 4. Foehn, P., & Scaramuzza, D. (2018, May). Onboard state dependent lqr for agile quadrotors.
 *    In 2018 IEEE International Conference on Robotics and Automation (ICRA) (pp. 6566-6572). IEEE.
 * @param state_estimate The current state estimate either coming from EKF or ground thrusth (pose, twist)
 * x,y,z,q.w,q.x, q.y, q,z, xdot, ydot, zdot, p,q,r
 * @param U Current previous control input
 * @return A matrix for Xdot = Ax + Bu; 
 * @author Sandesh Thapa, thapasandesh1@gmail.com
 */ 
 Eigen::Matrix<double, 12, 12> State_Dependent_LQR::A_LQR(   
      const Eigen::Matrix<double, 13, 1> & state_estimate, 
      const Eigen::Matrix<double, 4, 1> & u)
{
    double U1 = u(0);
    double U2 = u(1);
    double U3 = u(2);
    double U4 = u(3);
    Eigen::Quaterniond q; 
    q.w() = state_estimate(3); 
    q.x() = state_estimate(4); 
    q.y() = state_estimate(5); 
    q.z() = state_estimate(6); 

    // double _mass = 0.865; 
    // double _arm_len = 0.17; 

    setinertialparams( mass, arm_len);
    
    Eigen::Matrix<double, 3,1> Euler_ang = quaternionToEulerAnglesZYX(q); 
    // Euler_ang.setZero(); 

    // Euler_ang     = R_mat.eulerAngles(0,1,2); 
    double phi    = Euler_ang(0); 
    double theta  = Euler_ang(1); 
    double psi    = Euler_ang(2); 

//     % W_n = [1,      sin(X(4))*tan(X(5)),           cos(X(4))*tan(X(5)); 
// %        0,                cos(X(4)),                    -sin(X(4));
// %        0,       sin(X(4))/cos(X(5)),        cos(X(4))/cos(X(5)) ]; 
// %    
// % W_n_inv = [1,                   0,                    -sin(X(5));
// %            0,           cos(X(4)),           cos(X(5))*sin(X(4));
// %            0,          -sin(X(5)),           cos(X(5))*cos(X(4))]; 
// % 

    Eigen::Matrix<double, 3, 3 > W_n; 
    W_n.setZero(); 

    W_n << 1,   sin(phi)*tan(theta),  cos(phi)*tan(theta),
           0,              cos(phi),            -sin(phi),   
           0,   -sin(phi)/cos(theta), cos(phi)/cos(theta);


    Eigen::Matrix<double, 3, 3 > W_n_inv; 
    W_n.setZero(); 

    W_n_inv <<  1,                     0,             -sin(theta),
                0,              cos(phi),     cos(theta)*sin(phi),   
                0,            -sin(theta),     cos(phi)*cos(theta);

    Eigen::Matrix<double, 3,1> eta_dot; 
    eta_dot.setZero(); 

    Eigen::Matrix<double, 3,1> state_body_rates; 
    state_body_rates.setZero(); 
    state_body_rates(0) = state_estimate(10); 
    state_body_rates(1) = state_estimate(11); 
    state_body_rates(0) = state_estimate(12); 

    eta_dot = W_n*state_body_rates; 

    double phi_dot      = eta_dot(0); 
    double theta_dot    = eta_dot(1); 
    double psi_dot      = eta_dot(2); 

    Eigen::Matrix<double, 12, 12> A;
    A.setZero();

    A(0,6)  = 1.0; 
    A(1,7)  = 1.0;
    A(2,8)  = 1.0; 

    A(3,9)  = 1.0; 
    A(4,10) = 1.0;
    A(5,11) = 1.0; 

    A(6,3) = U1*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))/mass; 
    A(7,3) = -U1*(cos(phi)*cos(psi) + sin(psi)*sin(phi)*sin(theta))/mass; 
    A(8,3) = -U1*(cos(theta)*sin(phi))/mass;

    A(6,4) = U1*cos(phi)*cos(psi)*cos(theta)/mass; 
    A(7,4) = U1*cos(phi)*cos(theta)*sin(psi)/mass; 
    A(8,4) = - U1*cos(phi)*sin(theta)/mass; 

    A(6,5) = U1*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))/ mass; 
    A(7,5) = U1*(sin(psi)*sin(phi) + cos(phi)*cos(psi)*sin(theta))/ mass;

    A(10,9)     = -psi_dot*(I_xx - I_zz)/I_yy;

    A(9, 10)    = psi_dot*(I_yy - I_zz)/I_xx; 
    A(11,10)    = psi_dot*(I_xx - I_yy)/I_zz; 

    A(9,11)     = theta_dot*(I_yy - I_zz)/I_xx;
    A(10,11)    = -phi_dot*(I_xx - I_zz)/I_yy;
    A(11,11)    = theta_dot*(I_xx - I_yy)/I_zz;

    return A;
}


/**
 *  This functions returns the B matrix for quadrotor UAV for details view the following references: 
 * 1. Bouabdallah, S., Noth, A., & Siegwart, R. (2004, September).
 *    PID vs LQ control techniques applied to an indoor micro quadrotor.
 *    In 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems
 *    (IROS)(IEEE Cat. No. 04CH37566) (Vol. 3, pp. 2451-2456). IEEE.
 * 2. Bouabdallah, S., & Siegwart, R. (2007, October). Full control of a quadrotor.
 *     In 2007 IEEE/RSJ International Conference on Intelligent Robots and Systems (pp. 153-158). Ieee.
 * 3. Flightmare: A Flexible Quadrotor Simulator, 
 *    Song, Yunlong and Naji, Selim and Kaufmann, Elia and Loquercio, Antonio and Scaramuzza, Davide
 * 4. Foehn, P., & Scaramuzza, D. (2018, May). Onboard state dependent lqr for agile quadrotors.
 *    In 2018 IEEE International Conference on Robotics and Automation (ICRA) (pp. 6566-6572). IEEE.
 * @param state_estimate The current state estimate either coming from EKF or ground thrusth (pose, twist)
 * x,y,z,q.w,q.x, q.y, q,z, xdot, ydot, zdot, p,q,r
 * @param U Current previous control input
 * @return B matrix for Xdot = Ax + Bu; 
 * @author Sandesh Thapa, thapasandesh1@gmail.com
 */ 
 Eigen::Matrix<double, 12, 4> State_Dependent_LQR::B_LQR(   
        const Eigen::Matrix<double, 13, 1> & state_estimate, 
        const Eigen::Matrix<double, 4, 1> & u)
{
    double U1 = u(0);
    double U2 = u(1);
    double U3 = u(2);
    double U4 = u(3);
    Eigen::Quaterniond q; 
    q.w() = state_estimate(3); 
    q.x() = state_estimate(4); 
    q.y() = state_estimate(5); 
    q.z() = state_estimate(6); 

    Eigen::Matrix<double, 3,1> Euler_ang = quaternionToEulerAnglesZYX(q); 

    // Euler_ang     = R_mat.eulerAngles(0,1,2); 
    double phi    = Euler_ang(0); 
    double theta  = Euler_ang(1); 
    double psi    = Euler_ang(2); 

    Eigen::Matrix<double, 12, 4> B;
    B.setZero();

    // double _mass = 0.865; 
    // double _arm_len = 0.17; 

   setinertialparams( mass, arm_len);

    B(6,0)    = (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))/mass; 
    B(7,0)    = -(sin(phi)*cos(psi) + cos(phi)*sin(psi)*sin(theta))/mass; 
    B(8,0)    = cos(phi)*cos(theta)/mass; 

    B(9,1)    = 1/I_xx; 
    B(10,2)   = 1/I_yy; 
    B(11,3)   = 1/I_zz; 

    return B;
}


/**
 *  This functions solves the State Dependent LQR for current A and B matrix in real time
 *    for quadrotor UAV for details view the following references: 
 * 1. Bouabdallah, S., Noth, A., & Siegwart, R. (2004, September).
 *    PID vs LQ control techniques applied to an indoor micro quadrotor.
 *    In 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems
 *    (IROS)(IEEE Cat. No. 04CH37566) (Vol. 3, pp. 2451-2456). IEEE.
 * 2. Bouabdallah, S., & Siegwart, R. (2007, October). Full control of a quadrotor.
 *     In 2007 IEEE/RSJ International Conference on Intelligent Robots and Systems (pp. 153-158). Ieee.
 * 3. Flightmare: A Flexible Quadrotor Simulator, 
 *    Song, Yunlong and Naji, Selim and Kaufmann, Elia and Loquercio, Antonio and Scaramuzza, Davide
 * 4. Foehn, P., & Scaramuzza, D. (2018, May). Onboard state dependent lqr for agile quadrotors.
 *    In 2018 IEEE International Conference on Robotics and Automation (ICRA) (pp. 6566-6572). IEEE.
 * @param state_estimate The current state estimate either coming from EKF or ground thrusth (pose, twist)
 *  state_estimate = x,y,z,q.w,q.x, q.y, q,z, xdot, ydot, zdot, p,q,r
 *  @param reference_state The desired state to go for the quad 
 *  des_state = x,y,z,q.w,q.x, q.y, q,z, xdot, ydot, zdot, p,q,r
 * @param U Current previous control input
 * @return ULQR Control matrix for Xdot = Ax + Bu; 
 * @author Sandesh Thapa, thapasandesh1@gmail.com
 */ 
Eigen::Matrix<double, 4,1> State_Dependent_LQR::compute_Ulqr_12_states( const Eigen::Matrix<double, 13, 1> &reference_state,
                                                                            const Eigen::Matrix<double, 13, 1> & state_estimate )
{

  Eigen::Matrix<double, 4, 1> Ulqr; 
  Ulqr.setZero(); 

  Eigen::Matrix<double, 4, 1> Uref; 
  Uref.setZero(); 

  //  double _mass = 0.865; 
  //  double _arm_len = 0.17; 

   setinertialparams( mass, arm_len);

   std::cout << "Ixx" << I_xx << std::endl; 
 
  Uref(0) = mass*gravity;  

  // Solve the the lQ gain 

  Eigen::Matrix<double, 12, 12> A_curr; 
  Eigen::Matrix<double, 12,4> B_curr; 
  Eigen::Matrix<double, 12, 12> Q_;
  Eigen::Matrix<double, 4, 4> R_; 

  A_curr.setZero(); 
  B_curr.setZero(); 
  R_.setZero(); 
  Q_.setZero(); 

  Eigen::Matrix<double, 4, 1> U_prev; 
  U_prev = Uref; 

  A_curr = A_LQR(state_estimate, U_prev); 
  B_curr = B_LQR(state_estimate, U_prev); 

//   A_curr = A_LQR(reference_state, U_prev); 
//   B_curr = B_LQR(reference_state, U_prev); 

  // std::cout << "A matrix " << A_curr << std::endl; 
  // std::cout << "B matrix" << B_curr << std::endl; 

  R_(0,0)     = 0.001; 
  R_(1,1)     = 1000; 
  R_(2,2)     = 1000; 
  R_(3,3)     = 1000; 

  Q_(0,0)     = 1; 
  Q_(1,1)     = 1; 
  Q_(2,2)     = 450; 
  Q_(3,3)     = 1; 
  Q_(4,4)     = 1; 
  Q_(5,5)     = 450; 
  Q_(6,6)     = 1; 
  Q_(7,7)     = 1; 
  Q_(8,8)     = 1; 
  Q_(9,9)     = 100; 
  Q_(10,10)   = 100; 
  Q_(11,11)   = 1000; 
  
  // solve for lQ gain 

    State_Dependent_LQR::care_solver lqr_soln_;

    lqr_soln_ = care_soln(A_curr, B_curr, Q_, R_); 

    Eigen::Matrix<double, 12, 1> state_error; 
    state_error.setZero(); 

    // Eigen::Vector3d state_error_pos; 
    // state_error_pos(0) = reference_state(0) - state_estimate(0); 
    // state_error_pos(1) = reference_state(1) - state_estimate(1); 
    // state_error_pos(2) = reference_state(2) - state_estimate(2); 

    Eigen::Quaterniond q; 
    q.w() = state_estimate(3); 
    q.x() = state_estimate(4); 
    q.y() = state_estimate(5); 
    q.z() = state_estimate(6); 
    
    Eigen::Matrix<double, 3,1> Euler_ang = quaternionToEulerAnglesZYX(q); 
    double phi    = Euler_ang(0); 
    double theta  = Euler_ang(1); 
    double psi    = Euler_ang(2); 

    Eigen::Matrix<double, 3,1> Euler_ang_des; 

     Eigen::Quaterniond q_des; 
    q_des.w() = state_estimate(3); 
    q_des.x() = state_estimate(4); 
    q_des.y() = state_estimate(5); 
    q_des.z() = state_estimate(6); 

    Euler_ang_des     = quaternionToEulerAnglesZYX(q_des) ;
    double phi_des    = Euler_ang_des(0); 
    double theta_des  = Euler_ang_des(1); 
    double psi_des    = Euler_ang_des(2); 

    Eigen::Matrix<double, 4, 1> Ang_err; 

    Ang_err(0) = phi_des - phi; 
    Ang_err(1) = theta_des - theta; 
    Ang_err(2) = psi_des - psi; 

    Eigen::Matrix<double, 3, 3 > W_n; 
    // W_n.setZero(); 

    W_n << 1,   sin(phi)*tan(theta),  cos(phi)*tan(theta),
           0,              cos(phi),            -sin(phi),   
           0,   -sin(phi)/cos(theta), cos(phi)/cos(theta);


    Eigen::Matrix<double, 3, 3 > W_n_inv; 
    // W_n.setZero(); 

    W_n_inv <<  1,                     0,             -sin(theta),
                0,              cos(phi),     cos(theta)*sin(phi),   
                0,            -sin(theta),     cos(phi)*cos(theta);

    Eigen::Matrix<double, 3,1> eta_dot; 
    Eigen::Matrix<double, 3,1> eta_dot_des; 

    Eigen::Matrix<double, 3,1> state_body_rates; 
    state_body_rates.setZero(); 
    state_body_rates(0) = state_estimate(10); 
    state_body_rates(1) = state_estimate(11); 
    state_body_rates(0) = state_estimate(12); 

    eta_dot = W_n*state_body_rates; 


    Eigen::Matrix<double, 3,1> ref_body_rates; 
    ref_body_rates.setZero(); 
    ref_body_rates(0) = reference_state(10); 
    ref_body_rates(1) = reference_state(11); 
    ref_body_rates(0) = reference_state(12); 

    eta_dot_des = W_n*ref_body_rates; 
     
    double phi_dot      = eta_dot(0); 
    double theta_dot    = eta_dot(1); 
    double psi_dot      = eta_dot(2);

    double phi_dot_des      = eta_dot_des(0); 
    double theta_dot_des    = eta_dot_des(1); 
    double psi_dot_des      = eta_dot_des(2);


  state_error(0) =    reference_state(0) - state_estimate(0); 
  state_error(1) =    reference_state(1) - state_estimate(1);
  state_error(2) =    reference_state(2) - state_estimate(2);
  state_error(3) =    phi_des - phi; 
  state_error(4) =    theta_des - theta; 
  state_error(5) =    psi_des - psi; 
  state_error(6) =    reference_state(7) - state_estimate(7); 
  state_error(7) =    reference_state(8) - state_estimate(8); 
  state_error(8) =    reference_state(9) - state_estimate(9); 
  state_error(9) =    reference_state(10) - state_estimate(10);  //phi_dot_des - phi_dot; 
  state_error(10) =   reference_state(11) - state_estimate(11);//theta_dot_des - theta_dot; 
  state_error(11) =   reference_state(12) - state_estimate(12); // psi_dot_des - psi_dot; 

  Ulqr = Uref + lqr_soln_.K*(state_error); 
  
  U_prev = Ulqr; 

std::cout << "LQ Gain" << lqr_soln_.K << std::endl; 

std::cout << "U" << Ulqr << std::endl; 

  return Ulqr; 
}

/**
 * This  functions converts the current quaternion orientation to Euler angles in ZYX order 
 * For reference view: 
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param q Current quaternion pose
 * @return Euler_angles XYZ order
 * @author Sandesh Thapa, thapasandesh1@gmail.com
 */
Eigen::Vector3d State_Dependent_LQR::quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d euler_angles;
  // euler_angles(0) = atan2(
  //     2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(),
  //     q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  // euler_angles(1) = -asin(2.0 * q.x() * q.z() - 2.0 * q.w() * q.y());
  // euler_angles(2) = atan2(
  //     2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(),
  //     q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
      // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    euler_angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        euler_angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler_angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    euler_angles(2) = std::atan2(siny_cosp, cosy_cosp);
  return euler_angles;
}


int main()
{
   // Do State Dependent LQR here 
  double _mass = 0.73; 
  double _arm_len = 0.17;

   State_Dependent_LQR lqr(_mass, _arm_len); 
   lqr.setinertialparams( _mass, _arm_len);

   Eigen::Matrix<double, 13, 1> reference_state_ ;  
   reference_state_.setZero(); 
   reference_state_(0) = 0.0; 
   reference_state_(1) = 0.0; 
   reference_state_(2) = 0.0; 

   Eigen::Matrix<double, 13, 1> state_estimate_ ; 
   state_estimate_.setZero();
   state_estimate_(0) = 0.0;  

    Eigen::Matrix<double, 4,1> ULQR_12 = lqr.compute_Ulqr_12_states(reference_state_,state_estimate_);

    std::cout << "Current U: " << ULQR_12 << std::endl; 


    return 0; 
}
