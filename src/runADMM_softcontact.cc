/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

// #include "lcm/lcm-cpp.hpp"
// #include "robotlocomotion/robot_plan_t.hpp"
// #include "drake/lcmt_ddp_traj.hpp"

// #include "drake/common/drake_assert.h"
// #include "drake/common/find_resource.h"
// #include "drake/common/trajectories/piecewise_polynomial.h"
// #include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
// #include "drake/lcmt_iiwa_command.hpp"
// #include "drake/lcmt_iiwa_status.hpp"
// #include "drake/multibody/joints/floating_base_types.h"
// #include "drake/multibody/parsers/urdf_parser.h"
// #include "drake/multibody/rigid_body_tree.h"
// #include "drake/manipulation/util/world_sim_tree_builder.h"
// #include "drake/manipulation/planner/constraint_relaxing_ik.h"
// #include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

// #include "drake/lcmt_generic_string_msg.hpp"

#include <Eigen/Dense>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
// using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

/* ADMM trajectory generation */
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <string>
#include <list>

#include "config.h"
#include "spline.h"
#include "ilqrsolver_track.h"
#include "kuka_arm_track.h"
#include "SoftContactModel.h"
#include "KukaModel.h"
#include "models.h"


using namespace std;
using namespace Eigen;

/* ADMM trajectory generation */

static std::list< const char*> gs_fileName;
static std::list< std::string > gs_fileName_string;


/* ------------- Eigen print arguments ------------------- */
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
 /* ------------------------------------------------------- */


const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
// const int kNumJoints = 7;

// using trajectories::PiecewisePolynomial;
// typedef PiecewisePolynomial<double> PPType;
// typedef PPType::PolynomialType PPPoly;
// typedef PPType::PolynomialMatrix PPMatrix;

// using manipulation::planner::ConstraintRelaxingIk;
// using manipulation::kuka_iiwa::kIiwaArmNumJoints;
// using manipulation::util::WorldSimTreeBuilder;
// using manipulation::util::ModelInstanceInfo;
// using systems::RigidBodyPlant;

class RobotPlanRunner {
  public:
  void RunADMM(stateVec_t xinit, stateVec_t xgoal, stateVecTab_t xtrack) {
    struct timeval tbegin,tend;
    double texec = 0.0;
    commandVecTab_t u_0;
    double dt = TimeStep;
    unsigned int N = NumberofKnotPt;
    double tolFun = 1e-5;//1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-5;//relaxing default value: 1e-10; - gradient exit criteria
    unsigned int iterMax = 15; //DDP iteration max
    unsigned int ADMMiterMax = 5;
    // parameters for ADMM:
    double pos_weight; // penalty for position constraint
    double vel_weight; // penalty for velocity constraint
    double torque_weight; // penalty for torque constraint
    pos_weight = 0;
    vel_weight = 100;
    torque_weight = 0;

    // Initalize Primal and Dual variables
    // Primal
    stateVecTab_t xnew;
    commandVecTab_t unew;
    stateVecTab_t xbar;
    commandVecTab_t ubar;
    stateVecTab_t xbar_old; // "old" for last ADMM iteration 
    commandVecTab_t ubar_old; 
    
    // Dual
    stateVecTab_t x_lambda;
    commandVecTab_t u_lambda;

    stateVecTab_t x_temp;
    commandVecTab_t u_temp;
    stateVecTab_t x_temp2;
    commandVecTab_t u_temp2;
    projStateAndCommandTab_t xubar; // for projection
    vector<double> res_x;
    vector<double> res_u;
    vector<double> res_xlambda;
    vector<double> res_ulambda;
    vector<double> final_cost;
    res_x.resize(ADMMiterMax);
    res_u.resize(ADMMiterMax);
    res_xlambda.resize(ADMMiterMax);
    res_ulambda.resize(ADMMiterMax);
    final_cost.resize(ADMMiterMax+1);
    xbar.resize(N + 1);
    ubar.resize(N);
    xbar_old.resize(N + 1);
    ubar_old.resize(N);
    xubar.resize(N + 1);
    u_0.resize(N);
    x_lambda.resize(N + 1);
    u_lambda.resize(N);
    x_temp.resize(N+1);
    u_temp.resize(N);
    x_temp2.resize(N+1);
    u_temp2.resize(N);

    for(unsigned int k=0;k<N;k++){
      xbar[k].setZero();
      ubar[k].setZero();
      x_temp[k].setZero();
      u_temp[k].setZero();
      x_temp2[k].setZero();
      u_temp2[k].setZero();
      // x_lambda[k].setZero(); 
      // u_lambda[k].setZero();     
      u_0[k] << 0, 0, 0.2, 0, 0.2, 0, 0;
    }
    xbar[N].setZero();
    x_temp[N].setZero();
    x_temp2[N].setZero();
    // x_lambda[N].setZero();

    //======================================================================
    // Build wholebody and pass over to kukaArm
    // #if useILQRSolver

        /* -------------------- orocos kdl robot initialization-------------------------*/
        KUKAModelKDLInternalData robotParams;
        robotParams.numJoints = 7;
        robotParams.Kv = Eigen::MatrixXd(7,7);
        robotParams.Kp = Eigen::MatrixXd(7,7);

 
        KDL::Chain robot = KDL::KukaDHKdl();
        std::unique_ptr<KUKAModelKDL> kukaRobot = std::unique_ptr<KUKAModelKDL>(new KUKAModelKDL(robot, robotParams));

        //======================================

        #if WHOLE_BODY
            ContactModel::ContactParams cp_;
            cp_.E = 1000;
            cp_.mu = 0.5;
            cp_.nu = 0.4;
            cp_.R  = 0.005;
            cp_.R_path = 1000;
            cp_.Kd = 10;
            ContactModel::SoftContactModel contactModel(cp_);
            kukaRobot->initRobot();

        #endif

        // ILQRSolver::traj lastTraj;

        Eigen::VectorXd q_pos_init( (stateSize-3) / 2);
        Eigen::VectorXd q_vel_init( (stateSize-3) / 2);
        Eigen::VectorXd q_pos_goal( (stateSize-3) / 2);
        Eigen::VectorXd q_vel_goal( (stateSize-3) / 2);

        q_pos_init.setZero();
        q_pos_goal.setZero();
        q_vel_goal.setZero();
        q_vel_init.setZero();

        q_pos_init = xinit.head((stateSize-3)/2);
        q_vel_init = xinit.segment((stateSize-3)/2, (stateSize-3)/2);

        // std::cout << q_pos_init.transpose().format(CleanFmt) << std::endl;

        q_pos_goal = xgoal.head(stateSize/2);
        q_vel_goal = xgoal.segment((stateSize-3)/2, (stateSize-3)/2); 



        /*-----------------------------initial cartesian poses----------------------------------------------*/
        Eigen::Matrix<double,6,1> fkinit, fkgoal, fkstep;

        Eigen::Matrix<double,3,3> poseM;
        Eigen::Vector3d poseP;
        Eigen::Vector3d vel;
        Eigen::Vector3d accel;
        Eigen::VectorXd qdd(7);
        qdd.setZero();

        kukaRobot->getForwardKinematics(q_pos_init.data(), q_vel_init.data(), qdd.data(), poseM, poseP, vel, accel, false);
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(poseM);
        Eigen::Vector3d ea = m.eulerAngles(2, 1, 0); 

        fkinit << poseP(0), poseP(1), poseP(2), ea(0), ea(1), ea(2);

        std::cout << fkinit.transpose().format(CleanFmt) << std::endl;

        
        kukaRobot->getForwardKinematics(q_pos_goal.data(), q_vel_goal.data(), qdd.data(), poseM, poseP, vel, accel, false);
        m = Eigen::AngleAxisd(poseM);
        ea = m.eulerAngles(2, 1, 0); 

        fkgoal << poseP(0), poseP(1), poseP(2), ea(0), ea(1), ea(2);

        std::cout << fkgoal.transpose().format(CleanFmt) << std::endl;

        std::vector<Eigen::Matrix<double,6,1> > fk_ref(N+1);
        fk_ref[0] = fkinit;
        fk_ref[N] = fkgoal;



        /* Linear interpolation between two cartesian points */
        fkstep = (fkgoal - fkinit) / N;
        
        for(unsigned int i = 1; i < N; i++) 
        {
          fk_ref[i] = fk_ref[i-1] + fkstep;
          // cout << "fkref " << i <<": " << fk_ref[i].transpose() << endl;
        }


        /*----------------------warm-start-control-----------------------------*/
        // Get the gravity compensation for warm-start
        VectorXd q(9);
        VectorXd qd(9);
        q.setZero();

        // -----------------

        qd.setZero();
        Eigen::VectorXd gravityTorque(7);
        kukaRobot->getGravityVector(q_pos_init.data(), gravityTorque);

        std::cout << gravityTorque.transpose().format(CleanFmt) << std::endl;

        // MatrixX<double> M_ = totalTree_->massMatrix(cache_); // Inertial matrix

        // drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;

        // VectorX<double> gtau_wb = totalTree_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
        // VectorX<double> gtau_wb = totalTree_->inverseDynamics(cache_, f_ext, qd, false);

        // cout << "bias total" << endl << gtau_wb << endl;
 

    // #endif

    /*------------------initialize control input-----------------------*/
    // commandVecTab_t u_0;
    u_0.resize(N);

    for (unsigned i=0; i < N; i++)
    {
      u_0[i].head(7) = gravityTorque;
    }


    KukaArm_TRK KukaArmModel(dt, N, xgoal, xtrack, kukaRobot, contactModel, fk_ref);


    // Initialize ILQRSolver
    ILQRSolver_TRK::traj lastTraj;
    CostFunctionKukaArm_TRK costKukaArm_init(0,0,0);                                                  // only for initialization
    CostFunctionKukaArm_TRK costKukaArm_admm(pos_weight, vel_weight, torque_weight);                  // postion/velocity/torque weights

    ILQRSolver_TRK testSolverKukaArm(KukaArmModel,costKukaArm_admm,ENABLE_FULLDDP,ENABLE_QPBOX);

    ILQRSolver_TRK testSolverKukaArm_init(KukaArmModel,costKukaArm_init,ENABLE_FULLDDP,ENABLE_QPBOX); // only for initialization


    // Initialize Trajectory to get xnew with u_0 
    testSolverKukaArm_init.firstInitSolver(xinit, xgoal, xbar, ubar, u_0, N, dt, iterMax, tolFun, tolGrad);
    testSolverKukaArm_init.initializeTraj();


    // xnew = testSolverKukaArm.updatedxList;
    // unew = u_0;
    // final_cost[0] = accumulate(testSolverKukaArm.costList.begin(), testSolverKukaArm.costList.end(), 0.0);
    lastTraj = testSolverKukaArm_init.getLastSolvedTrajectory();
    xnew = lastTraj.xList;
    unew = lastTraj.uList;
    final_cost[0] = lastTraj.finalCost;

    for(unsigned int k=0;k<N;k++){
      x_lambda[k] = xnew[k] - xbar[k];
      u_lambda[k] = unew[k] - ubar[k];
    }
    x_lambda[N] = xnew[N] - xbar[N];
   
    // Run ADMM
    cout << "\n=========== begin ADMM ===========\n";
    gettimeofday(&tbegin,NULL);

    for (unsigned int i = 0; i < ADMMiterMax; i++)
    {// TODO: Stopping criterion is needed
      for (unsigned int k = 0;k < N; k++)
      {
        x_temp[k] = xbar[k] - x_lambda[k];
        u_temp[k] = ubar[k] - u_lambda[k];
      }
      x_temp[N] = xbar[N] - x_lambda[N];
      
      cout << "\n=========== ADMM iteration " << i+1 << " ===========\n";

      // iLQR solver block
      testSolverKukaArm.firstInitSolver(xinit, xgoal, x_temp, u_temp, unew, N, dt, iterMax, tolFun, tolGrad);
      testSolverKukaArm.solveTrajectory();
      // testSolverKukaArm.initializeTraj();
      lastTraj = testSolverKukaArm.getLastSolvedTrajectory();

      /* ADMM update */
      xnew = lastTraj.xList;
      unew = lastTraj.uList;
      
      // Projection block to feasible sets (state and control contraints)
      xbar_old = xbar;
      ubar_old = ubar;
      for (unsigned int k = 0;k < N; k++)
      {
        x_temp2[k] = xnew[k] + x_lambda[k];
        u_temp2[k] = unew[k] + u_lambda[k];
      }
      x_temp2[N] = xnew[N] + x_lambda[N];

      xubar = projection(x_temp2, u_temp2);



      // std::cout << xubar.at(0) << std::endl;
      // Dual variables update
      for (unsigned int j = 0;j < N; j++)
      {
        xbar[j] = xubar[j].head(stateSize);
        ubar[j] = xubar[j].tail(commandSize);
        // cout << "u_bar[" << j << "]:" << ubar[j].transpose() << endl;
        x_lambda[j] += xnew[j] - xbar[j];
        u_lambda[j] += unew[j] - ubar[j];
        // cout << "u_lambda[" << j << "]:" << u_lambda[j].transpose() << endl;

        // Save residuals for all iterations
        res_x[i] += (xnew[j] - xbar[j]).norm();
        res_u[i] += (unew[j] - ubar[j]).norm();

        res_xlambda[i] += vel_weight*(xbar[j] - xbar_old[j]).norm();
        res_ulambda[i] += 0*(ubar[j] - ubar_old[j]).norm();
      }

      xbar[N] = xubar[N].head(stateSize);
      x_lambda[N] += xnew[N] - xbar[N];

      res_x[i] += (xnew[N] - xbar[N]).norm();
      res_xlambda[i] += 0*(xbar[N] - xbar_old[N]).norm();

      // get the cost without augmented Lagrangian terms
      testSolverKukaArm_init.firstInitSolver(xinit, xgoal, xbar, ubar, unew, N, dt, iterMax, tolFun, tolGrad);
      testSolverKukaArm_init.initializeTraj();
      lastTraj = testSolverKukaArm_init.getLastSolvedTrajectory();
      final_cost[i+1] = lastTraj.finalCost;
      // cout << "checkpoint 3" << endl;
    }

    gettimeofday(&tend,NULL);    

    testSolverKukaArm.firstInitSolver(xinit, xgoal, xbar, ubar, unew, N, dt, iterMax, tolFun, tolGrad);
    testSolverKukaArm.initializeTraj();
    xnew = testSolverKukaArm.updatedxList;
    

    joint_state_traj.resize(N+1);
    joint_state_traj_interp.resize(N*InterpolationScale+1);
    for(unsigned int i=0;i<=N;i++){
      joint_state_traj[i] = xnew[i];
    }
    torque_traj = unew;

    //linear interpolation to 1ms
    for(unsigned int i=0;i<stateSize;i++){
      for(unsigned int j=0;j<N*InterpolationScale;j++){
       unsigned int index = j/10;
       joint_state_traj_interp[j](i,0) =  joint_state_traj[index](i,0) + (static_cast<double>(j)-static_cast<double>(index*10.0))*(joint_state_traj[index+1](i,0) - joint_state_traj[index](i,0))/10.0;
      }
      joint_state_traj_interp[N*InterpolationScale](i,0) = joint_state_traj[N](i,0);
    }

    texec=(static_cast<double>(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    // texec /= Num_run;

    cout << endl;
    // cout << "Number of iterations: " << lastTraj.iter + 1 << endl;
    cout << "Final cost: " << lastTraj.finalCost << endl;
    // cout << "Final gradient: " << lastTraj.finalGrad << endl;
    // cout << "Final lambda: " << lastTraj.finalLambda << endl;
    // cout << "Execution time by time step (second): " << texec/N << endl;
    // cout << "Execution time per iteration (second): " << texec/lastTraj.iter << endl;
    cout << "Total execution time of the solver (second): " << texec << endl;
    // cout << "\tTime of derivative (second): " << lastTraj.time_derivative.sum() << " (" << 100.0*lastTraj.time_derivative.sum()/texec << "%)" << endl;
    // cout << "\tTime of backward pass (second): " << lastTraj.time_backward.sum() << " (" << 100.0*lastTraj.time_backward.sum()/texec << "%)" << endl;



    // for(unsigned int i=0;i<=N;i++){
    //   cout << "lastTraj.xList[" << i << "]:" << lastTraj.xList[i].transpose() << endl;
    // }
    // cout << "--------- final joint torque trajectory ---------" << endl;
    
    // for(unsigned int i=0;i<=N;i++){
    //   cout << "lastTraj.uList[" << i << "]:" << lastTraj.uList[i].transpose() << endl;
    // }

    cout << "lastTraj.xList[" << N << "]:" << xnew[N].transpose() << endl;
    cout << "lastTraj.uList[" << N-1 << "]:" << unew[N-1].transpose() << endl;

    cout << "lastTraj.xList[0]:" << xnew[0].transpose() << endl;
    cout << "lastTraj.uList[0]:" << unew[0].transpose() << endl;

    // saving data file
    for(unsigned int i=0;i<N;i++){
      saveVector(joint_state_traj[i], "joint_trajectory_ADMM");
      saveVector(torque_traj[i], "joint_torque_command_ADMM");
    }
    saveVector(xnew[N], "joint_trajectory_ADMM");

    for(unsigned int i=0;i<=N*InterpolationScale;i++){
      saveVector(joint_state_traj_interp[i], "joint_trajectory_interpolated_ADMM");
    }

    cout << "-------- ADMM Trajectory Generation Finished! --------" << endl;
    // for(unsigned int i=N-50;i<N;i++)
    // {
    //   // cout << "xnew[" << i << "]:" << xnew[i].transpose() << endl;
    //   cout << "unew[" << i << "]:" << unew[i].transpose() << endl;
    //   // cout << "res_xlambda[" << i << "]:" << res_xlambda[i] << " ";
    // }

    for(unsigned int i=0;i<ADMMiterMax;i++)
    {
      cout << "res_x[" << i << "]:" << res_x[i] << endl;
      // cout << "res_xlambda[" << i << "]:" << res_xlambda[i] << " ";
    }
    cout << endl;
    for(unsigned int i=0;i<=ADMMiterMax;i++)
    {
      // cout << "res_u[" << i << "]:" << res_u[i] << endl;
      // cout << "res_xlambda[" << i << "]:" << res_xlambda[i] << " ";
      cout << "final_cost[" << i << "]:" << final_cost[i] << endl;
    }


  }

  projStateAndCommandTab_t projection(const stateVecTab_t& xnew, const commandVecTab_t& unew){
      projStateAndCommandTab_t xubar;
      xubar.resize(NumberofKnotPt+1);

      for(unsigned int i=0;i<NumberofKnotPt+1;i++){
        for(unsigned int j=0;j<stateSize+commandSize;j++){
        if(SOFT_CONTACT)
        {
          if(j < (stateSize-3)/2) {//postion constraints
            if(xnew[i](j,0) > 2.0){
              xubar[i](j,0) = 2.0;
            }
            else if(xnew[i](j,0) < -2.0){
              xubar[i](j,0) = -2.0;
            }
            else{
              xubar[i](j,0) = xnew[i](j,0);
            }
          }

          else if(j >= (stateSize-3)/2 && j < stateSize-3){//velocity constraints
            if(xnew[i](j,0) > 0.5){
              xubar[i](j,0) = 0.5;
            }
            else if(xnew[i](j,0) < -0.5){
              xubar[i](j,0) = -0.5;
            }
            else{
              xubar[i](j,0) = xnew[i](j,0);
            }
          }

          else if(j >= (stateSize-3) && j < stateSize){//contact force constraints
            //TODO: fricition cone or other constraints related to contact force
          }

          else
          {//torque constraints
              if(unew[i](j-stateSize,0) > 30){
                xubar[i](j,0) = 30;
              }
              else if(unew[i](j-stateSize,0) < -30){
                xubar[i](j,0) = -30;
              }
              else{
                xubar[i](j,0) = unew[i](j-stateSize,0);
              }
            }
            // std::cout << j << std::endl;
        }
        ///NOT SOFT CONTACT (FOR DEBUG ADMM)
        else
        {
          if(j < stateSize/2){//postion constraints
            if(xnew[i](j,0) > 2.0){
              xubar[i](j,0) = 2.0;
            }
            else if(xnew[i](j,0) < -2.0){
              xubar[i](j,0) = -2.0;
            }
            else{
              xubar[i](j,0) = xnew[i](j,0);
            }
          }

          else if(j >= stateSize/2 && j < stateSize){//velocity constraints
            if(xnew[i](j,0) > 0.5){
              xubar[i](j,0) = 0.5;
            }
            else if(xnew[i](j,0) < -0.5){
              xubar[i](j,0) = -0.5;
            }
            else{
              xubar[i](j,0) = xnew[i](j,0);
            }
          }

          else{//torque constraints
              if(unew[i](j,0) > 30){
                xubar[i](j,0) = 30;
              }
              else if(unew[i](j,0) < -30){
                xubar[i](j,0) = -30;
              }
              else{
                xubar[i](j,0) = unew[i](j,0);
              }
          }
        }
      }
      }
      return xubar;
  }

  void saveVector(const Eigen::MatrixXd & _vec, const char * _name){
      std::string _file_name = UDP_TRAJ_DIR;
      _file_name += _name;
      _file_name += ".csv";
      clean_file(_name, _file_name);

      std::ofstream save_file;
      save_file.open(_file_name, std::fstream::app);
      for (int i(0); i < _vec.rows(); ++i){
          save_file<<_vec(i,0)<< "\t";
      }
      save_file<<"\n";
      save_file.flush();
      save_file.close();
  }

  void saveValue(double _value, const char * _name){
      std::string _file_name = UDP_TRAJ_DIR;
      _file_name += _name;
      _file_name += ".csv";
      clean_file(_name, _file_name);

      std::ofstream save_file;
      save_file.open(_file_name, std::fstream::app);
      save_file<<_value <<"\n";
      save_file.flush();
  }

  void clean_file(const char * _file_name, std::string & _ret_file){
      std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
      if(gs_fileName_string.end() == iter){
          gs_fileName_string.push_back(_file_name);
          remove(_ret_file.c_str());
      }
  }

 private:
  // lcm::LCM lcm_;
  // lcmt_ddp_traj ddp_traj_;

  //UDP parameters
  stateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  stateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;
  // unsigned int traj_knot_number_ = 0;
};

int do_main() {
  RobotPlanRunner runner;

  stateVec_t xinit,xgoal;
  stateVecTab_t xtrack;
  xtrack.resize(NumberofKnotPt+1);

  // xinit << 0, 0.6, 0, -1.75, 0, 1.0, 0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0; //grasp ready position
  // xgoal << -0.00174812, 0.737294, 0.000945954, -0.50355, 0.00285474, -0.593983, -0.00240676, 0.00076296, 0.045792, 0.0021688, -0.0319025, 0.00242267, 0.0227341, 0.00134487;

  xinit << 0, 0.5, 0, 1.0, 0, 0.5, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0;
  xgoal << 1.14, 1.93, -1.48, -1.78, 0.31, 0.13, 1.63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  for(unsigned int i=0;i<NumberofKnotPt+1;i++){
      xtrack[i].setZero();
  } 

  // Debugging for ILQR_TRACKING

  runner.RunADMM(xinit, xgoal, xtrack);

  return 0;
}



int main() {
  return do_main();
}