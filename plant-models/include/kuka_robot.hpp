#ifndef KUKA_ROBOT_HPP
#define KUKA_ROBOT_HPP

#include <iostream>
#include <Eigen/Dense>
#include "modern_robotics.h"
// # define M_PI           3.14159265358979323846  /* pi */

namespace models {

class KUKA {

public:
	KUKA() {

		Slist.resize(6,7);
		M.resize(4,4);

		Eigen::Vector3d w1(0,0,1);
		Eigen::Vector3d w2(0,1,0);
		Eigen::Vector3d w3(0,0,1);
		Eigen::Vector3d w4(0,-1,0);
		Eigen::Vector3d w5(0,0,1);
		Eigen::Vector3d w6(0,1,0);
		Eigen::Vector3d w7(0,0,1);

		Eigen::Vector3d q1(0,0,(0.2025+0.1575));
		Eigen::Vector3d q2(0,0, (0.2025+0.1575));
		Eigen::Vector3d q3(0,0,0.2025+0.1575);
		Eigen::Vector3d q4(0,0,0.2025+0.42+0.1575);
		Eigen::Vector3d q5(0,0,0.2025+0.42+0.1575);
		Eigen::Vector3d q6(0,0,0.2025+0.42+0.4+0.1575);
		// Eigen::Vector3d q7(0,0,0.2025+0.42+0.4+0.126+0.1575);
		Eigen::Vector3d q7(0,0,0.2025+0.42+0.4+0.241+0.1575);



	    M << 1, 0, 0, 0,
	    		0, 1, 0, 0,
	    		0, 0, 1, 0.2025+0.42+0.4+0.241+0.1575,
	    		0, 0, 0, 1;

	    double h = 0;
	    Eigen::VectorXd S1 = mr::ScrewToAxis(q1,w1, h);
	    Eigen::VectorXd S2 = mr::ScrewToAxis(q2,w2, h);
	    Eigen::VectorXd S3 = mr::ScrewToAxis(q3,w3, h);
	    Eigen::VectorXd S4 = mr::ScrewToAxis(q4,w4, h);
	    Eigen::VectorXd S5 = mr::ScrewToAxis(q5,w5, h);
	    Eigen::VectorXd S6 = mr::ScrewToAxis(q6,w6, h);
	    Eigen::VectorXd S7 = mr::ScrewToAxis(q7,w7, h);

	    Slist << S1, S2, S3, S4, S5, S6, S7;
	}

	~KUKA(){}
	void getSlist(Eigen::MatrixXd* s_list) {
		*(s_list) = Slist;
	}

	void getM(Eigen::MatrixXd* M_) {
		*(M_) = M;
	}


	// get FK
	std::vector<Eigen::Matrix3d> getFK(const Eigen::MatrixXd& joint_positions) {
		std::vector<Eigen::Matrix3d> fk(joint_positions.cols());

		for (int i = 0;i < joint_positions.cols(); i++) {
			fk.at(i) = mr::FKinSpace(M, Slist, joint_positions.col(i));
		}

		return fk;
	}

private:
	Eigen::MatrixXd Slist;
	Eigen::MatrixXd M;
};


}

#endif // KUKA_ROBOT_HPP