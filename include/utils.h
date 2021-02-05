#ifndef ADMM_UTILS_HPP
#define ADMM_UTILS_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <sys/time.h>
#include <cstdarg>
#include <cmath>

// std::vector<Eigen::MatrixXd> admm::utils::generateLissajousTrajectories(const Eigen::MatrixXd& R, double z, int n, int m, double r1, double r2, int N, double Tf); 


namespace admm 
{
    namespace utils
    {

        std::vector<Eigen::MatrixXd> generateLissajousTrajectories(const Eigen::MatrixXd& R, double z, int n, int m, double r1, double r2, int N, double Tf) 
        {
            double timegap = Tf / (N);
            std::vector<Eigen::MatrixXd> traj(N + 1);
            Eigen::Vector3d p;
            Eigen::Matrix<double, 4, 4> T;
            T(3,3) = 1;

            double t = 0.0;

            for (int i = 0;i < N+1; i++) 
            {
                p(0) = r1 * std::cos(n * t); p(1) = r2 * std::sin(m * t); p(2) = z;
                T.block(0,0,3,3) = R;
                T.block(0,3,3,1) = p;
                traj.at(i) = T;
                t = t + timegap;
            }
            return traj;

        }

    }
}

#endif // CONFIG_H
