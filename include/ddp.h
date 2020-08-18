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

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector2d;
using Eigen::Vector3d;

#include "config.h"
#include "spline.h"
#include "ilqrsolver.h"
#include "kuka_arm.h"
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

  // void saveVector(const Eigen::MatrixXd & _vec, const char * _name) 
  // {
  //     // std::string _file_name = UDP_TRAJ_DIR;
  //     _file_name += _name;
  //     _file_name += ".csv";
  //     clean_file(_name, _file_name);

  //     std::ofstream save_file;
  //     save_file.open(_file_name, std::fstream::app);
  //     for (int i(0); i < _vec.rows(); ++i)
  //     {
  //         save_file<<_vec(i,0)<< "\t";
  //     }
  //     save_file<<"\n";
  //     save_file.flush();
  //     save_file.close();
  // };


  // void clean_file(const char * _file_name, std::string & _ret_file)
  // {
  //     std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
  //     if (gs_fileName_string.end() == iter)
  //     {
  //         gs_fileName_string.push_back(_file_name);
  //         remove(_ret_file.c_str());
  //     }
  // };

private:
  stateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  stateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;

protected:
  optimizer::ILQRSolver::traj lastTraj;

};





