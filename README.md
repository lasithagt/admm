# Differential Dynamical Programming with Soft Contact Dynamics
This code is primarily uses DDP as the optimizer with autodiff for real-time implementation. The analytical model is derived using RobCoGen. [control-toolbox](https://github.com/ethz-adrl/control-toolbox.git) is used to use Autodiff functionality for real-time implentation. <br>
This repo uses DDP (Differential Dynamic Programming) + Soft-Contact inside a ADMM (Alternating Direction Multiplier Method).

### Structure
- Standalone DDP code (C++)
- ADMM integration with constraints and DDP
- Model Predictive Control (MPC)

### Dependencies
Dynamics of the robot are computed using
- orocos KDL
- [control-toolbox](https://github.com/ethz-adrl/control-toolbox.git)


### Installation
```
mkdir build & cd build
ccmake ..
```
To use the eigen library installed in the system, enable the the option ```USE_SYSTEM_EIGEN```. To build from source.
```
make
```

### How to use
- Scripts for executables can be found in ```src```
- Files specific to ADMM - ```admm```
- Files specific to DDP - ```ddp```

- Following executables are generated
  - admm-contact *(admm with orocos KDL dynamics, finite difference)*
  - admm-contact-rcg *(admm with RobCoGen dynamics, AutoDiff)*
  - admm-mpc-contact - *(MPC - admm with orocos KDL dynamics, finite difference)*
  - admm-mpc-contact-rcg - *(MPC - admm with RobCoGen dynamics, AutoDiff)*

### Aknowledgement and Contact
Lasitha Wijayarathne - *lasitha@gatech.edu*

### Citation
