
#include"cnpy.h"
#include<complex>
#include<cstdlib>
#include<iostream>
#include<map>
#include<string>
#include <Eigen/Dense>

#include <memory>
#include <unsupported/Eigen/CXX11/Tensor>


const int Nx = 128;
const int Ny = 64;
const int Nz = 32;

int main() {

	Eigen::MatrixXd d(2,2);
	d << 2,3,1,3;

	Eigen::Tensor<double,3> m;
	m.resize(3,5,4);            				   //Initialize
	m.setZero();                                 //Set random values 

	int k = 0;
	for (int i = 0; i < 5;i++) {
		for (int j = 0; j < 3;j++) {
			k++;
			m(j, i, 0) = k;
		} 
	}



	// Eigen::array<long,3> offset = {0,0,0};         //Starting point
	// Eigen::array<long,3> extent = {1,4,4};         //Finish point 


	// // Eigen::Tensor<double,2> n = m.slice(offset, extent);

	// std::cout <<  m.slice(offset, extent).reshape(Eigen::array<int, 2>{4,4}) << std::endl;  //Reshape the slice into a 10x10 matrix.

	// std::cout << "a" << std::endl << a << std::endl;
	// std::cout << "slice" << std::endl << slice << std::endl;

    //set random seed so that result is reproducible (for testing)
    srand(0);
    //create random data
    Eigen::Matrix3d mat;
    mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    std::vector<double> data(9);
    // for(int i = 0;i < Nx*Ny*Nz;i++) data[i] = std::compdouble>(rand(),rand());

    memcpy(data.data(), mat.data(), sizeof(double) * 9);

    //save it to file
    cnpy::npy_save("../data/3dtest.npy", m.data(), {4,5,3}, "w");
}