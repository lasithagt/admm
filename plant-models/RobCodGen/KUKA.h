/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "generated/declarations.h"
#include "generated/jsim.h"
#include "generated/jacobians.h"
#include "generated/traits.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"

// define namespace and base
#define ROBCOGEN_NS Kuka
#define TARGET_NS KUKA

// define the links
#define CT_BASE fr_base0
#define CT_L0 fr_link1
#define CT_L1 fr_link2
#define CT_L2 fr_link3
#define CT_L3 fr_link4
#define CT_L4 fr_link5
#define CT_L5 fr_link6
#define CT_L6 fr_link7

// define single end effector (could also be multiple)
#define CT_N_EE 1
#define CT_EE0 fr_ee
#define CT_EE0_IS_ON_LINK 7
#define CT_EE0_FIRST_JOINT 0
#define CT_EE0_LAST_JOINT 6

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>


