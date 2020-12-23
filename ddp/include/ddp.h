#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <string>
#include <list>

#include <Eigen/Dense>


#include "config.h"
#include "ilqrsolver.h"
#include "RobotDynamics.hpp"
#include "SoftContactModel.h"
#include "KukaModel.h"
#include "models.h"


using namespace std;
using namespace Eigen;

/* DDP trajectory generation */

// static std::list< const char*> gs_fileName;
// static std::list< std::string > gs_fileName_string;


/* -------------------- Soft_contact_state = 17(14+3) ------------------------*/
class DDP 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DDP();

  void run(stateVec_t xinit, stateVec_t xgoal, stateVecTab_t xtrack);



private:
  stateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  stateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;

protected:
  optimizer::ILQRSolver::traj lastTraj;

};





