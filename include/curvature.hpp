
#ifndef CURVATURE_HPP
#define CURVATURE_HPP

#include <iostream>
#include <fstream>
#include <cmath>

#include <math.h>       /* pow */

#include <vector>
#include <stdio.h>
#include <string>

#include <Eigen/Dense>

class Curvature {

public:
  Curvature(){}
  ~Curvature(){}

  void circumcenter(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C, bool isR, double& R, Eigen::Vector3d* k) {
    // Center and radius of the circumscribed circle for the triangle ABC
    // A,B,C  3D coordinate vectors for the triangle corners
    // R      Radius
    // M      3D coordinate vector for the center
    // k      Vector of length 1/R in the direction from A towards M
    //        (Curvature vector)


    Eigen::Vector3d D = (B - A).cross(C - A);

    if (isR == true) {
      // slightly faster if only R is required
      R = (B - C).norm() * (A - C).norm() * (A - B).norm() / 2 / D.norm();
      return;
    }

    Eigen::Vector3d G = (pow((A - C).norm(), 2) * D.cross(B - A) - pow((A - B).norm(), 2) * D.cross(C - A)) / pow(D.norm(), 2) / 2;

    auto M = A + G;
    R = G.norm();  // Radius of curvature

    if (R == 0) {
      *k = G;
    } else {
      *k = G.transpose() / pow((R), 2);   // Curvature vector
    }

  }

  void curvature(const Eigen::MatrixXd& X, 
    Eigen::VectorXd& L, Eigen::VectorXd& R, Eigen::MatrixXd& k) {
    // Radius of curvature and curvature vector for 2D or 3D curve
    //  [L,R,Kappa] = curvature(X)
    //   X:   2 or 3 column array of x, y (and possibly z) coordiates
    //   L:   Cumulative arc length
    //   R:   Radius of curvature
    //   k:   Curvature vector


    int N = X.rows();
    int dims = X.cols();
    Eigen::MatrixXd X_;
    X_.setZero();

    if (dims == 2) {
      X_.resize(N, 2);
    } else {
      X_.resize(N, dims);
    }


    double temp_R = 0.0;
    Eigen::Vector3d temp_k;

    for (int i = 1; i < N - 1; i++) 
    {
      circumcenter(X.row(i), X.row(i - 1), X.row(i + 1), false, temp_R, &temp_k);
      R(i) = temp_R;
      k.row(i) = temp_k;
      L(i) = L(i - 1) + (X.row(i) - X.row(i - 1)).norm();
    }

    R(0) = R(1);
    R(N - 1) = R(N - 2);

    k.row(0) = k.row(1);
    k.row(N - 1) = k.row(N - 2);

    L(N - 1) = L(N - 2) + (X.row(N - 1) - X.row(N - 2)).norm();

    // for (int i=0;i<20;i++) {
    //   R(i) = 1;
    // }

  }


};


#endif

