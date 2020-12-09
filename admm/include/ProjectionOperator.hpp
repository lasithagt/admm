#include <Eigen/Dense>
#include "config.h"
#include "admm.hpp"

/* projection operator */
class ProjectionOperator {


public:

	/* Projection Block 
	Projects the states and commands to be within bounds
	*/
	Eigen::MatrixXd projection(const stateVecTab_t& xnew, const Eigen::MatrixXd& cnew, const commandVecTab_t& unew, const ADMM::Saturation& L) {

	    double mu = 0.3;
	    for(int i = 0;i < N ; i++) {

	        for (int j = 0;j < stateSize + commandSize + 2; j++) {
	            /* postion + velocity + force constraints */
	            if(j < stateSize) { 
	                if (xnew(j,i) > L.stateLimits(1, j)) {
	                    xubar(j,i) = L.stateLimits(1, j);
	                }
	                else if(xnew(j,i) < L.stateLimits(0, j)) {
	                    xubar(j,i) = L.stateLimits(0, j);
	                }
	                else {
	                    xubar(j,i) = xnew(j,i);
	                }
	            /* contact constraints */
	            } else if(j == stateSize) { 

	                if((cnew(0, i)) > mu * std::abs(cnew(1, i))) {
	                    xubar(j,i) = mu * std::abs(cnew(1, i));
	                } else {
	                    xubar(j,i) = cnew(0, i);
	                }

	            } else if(j == stateSize + 1) {
	                xubar(j,i) = cnew(1, i);
	            /* torque constraints */
	            } else { 

	                if(unew(j - stateSize - 2, i) > L.controlLimits(1, j - stateSize - 2)) {
	                    xubar(j, i) = L.controlLimits(1, j - stateSize - 2);
	                }
	                else if(unew(j - stateSize - 2, i) < L.controlLimits(0, j - stateSize - 2)) {
	                    xubar(j, i) = L.controlLimits(0, j - stateSize - 2);
	                }
	                else {
	                    xubar(j, i) = unew(j-stateSize-2, i);
	                }
	            }

	        }
	    }

	    return xubar;

};