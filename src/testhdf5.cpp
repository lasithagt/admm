
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

	Eigen::Tensor<double,3> m(3,10,10);            //Initialize
	m.setRandom();                                 //Set random values 

	Eigen::array<long,3> offset = {1,0,0};         //Starting point
	Eigen::array<long,3> extent = {1,2,2};       //Finish point 


	// Eigen::Tensor<double,2> n = m.slice(offset, extent);

	std::cout <<  m.slice(offset, extent).reshape(Eigen::array<long,2>{2,2}) << std::endl;  //Reshape the slice into a 10x10 matrix.

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
    cnpy::npy_save("./standalone⁩arr1.npy",m.data(),{3,10,10},"w");
}