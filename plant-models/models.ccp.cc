// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>

#include "models.h"

namespace KDL
{


Chain KukaDHKdl()
{
    Chain KukaDHKdl_;
	
	//joint 0
	KukaDHKdl_.addSegment(Segment(Joint(Joint::None),
				  Frame::DH_Craig1989(0, 0, 0.36, 0)));

	//joint 1
	KukaDHKdl_.addSegment(Segment(Joint(Joint::RotZ,1,0,0,0,100000),
				  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
				  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(3.9,
												 Vector::Zero(),
												 RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));
				   
	//joint 2 
	KukaDHKdl_.addSegment(Segment(Joint(Joint::RotZ,1,0,0,0,100000),
				  Frame::DH_Craig1989(0.0, -1.5707963, 0.42, 0.0),
				  Frame::DH_Craig1989(0.0, -1.5707963, 0.42, 0.0).Inverse()*RigidBodyInertia(4.5,
												   Vector(0.0,-0.3120511,-0.0038871),
												   RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));
				  
	//joint 3
	KukaDHKdl_.addSegment(Segment(Joint(Joint::RotZ,1,0,0,0,100000),
				  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
				  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2.45,
												   Vector(0.0,-0.0015515,0.0),
												   RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));
				  
	//joint 4
	KukaDHKdl_.addSegment(Segment(Joint(Joint::RotZ,1,0,0,0,100000),
				  Frame::DH_Craig1989(0.0, 1.5707963, 0.4, 0.0),
				  Frame::DH_Craig1989(0.0, 1.5707963, 0.4, 0.0).Inverse()*RigidBodyInertia(2.61,
												   Vector(0.0,0.5216809,0.0),
												   RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));
				  
	//joint 5
	KukaDHKdl_.addSegment(Segment(Joint(Joint::RotZ,1,0,0,0,100000),
				  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
				  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(3.41,
												   Vector(0.0,0.0119891,0.0),
												   RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));
				  
	//joint 6
	KukaDHKdl_.addSegment(Segment(Joint(Joint::RotZ,1,0,0,0,100000),
				  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
				  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(3.38,
												   Vector(0.0,0.0080787,0.0),
												   RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
	//joint 7
	KukaDHKdl_.addSegment(Segment(Joint(Joint::RotZ,1,0,0,0,100000),
				   Frame::DH_Craig1989(0.0, 0, 0.126, 0.0),
				   Frame::DH_Craig1989(0.0, 0, 0.126, 0.0).Inverse()*RigidBodyInertia(0.35,
												   Vector::Zero(),
												   RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));
    return KukaDHKdl_;
}

}
