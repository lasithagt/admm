cmake_minimum_required(VERSION 3.15)

project(DDP-ADMM)
SET (CMAKE_CXX_COMPILER "/usr/bin/g++")

include(GenerateExportHeader)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set (CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)


# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(DDP_SYSTEM_EIGEN "Use system-installed version of Eigen" OFF)
# option(DDP_BUILD_EXAMPLES "Build examples" ON)

set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(DDP_SYSTEM_EIGEN)
    find_package(Eigen)
    if(EIGEN_FOUND)
        message(STATUS "Eigen found: ${EIGEN_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Eigen NOT found!")
    endif()
endif()

# find_package(Eigen3 REQUIRED)

find_package(blasfeo QUIET)
find_package(hpipm QUIET)

find_package(OROCOS_KDL)
find_package(ModernRoboticsCpp)
find_package(CNPY)
find_package(ct_core)
find_package(ct_rbd)
find_package(ct_optcon)



if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "-march=native -mtune=native -mavx2 -mfma")
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread")


include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${OROCOS_KDL_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/plant-models)

# find ModernRobotics package
add_subdirectory(plant-models)
add_subdirectory(ddp)
add_subdirectory(ik-solvers)
add_subdirectory(admm)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")



# EXECUTABLES
#######################################################################################

add_executable(admm-contact src/OPT_solvers.cpp src/main_admm.cpp)
target_include_directories(admm-contact PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(admm-contact orocos-kdl admm-solver kuka-models ModernRoboticsCpp ik-solvers ct_core ct_rbd ct_optcon)

add_executable(admm-contact-rcg src/OPT_solvers.cpp src/main_admm_rcg.cpp)
target_include_directories(admm-contact-rcg PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(admm-contact-rcg orocos-kdl admm-solver kuka-models ModernRoboticsCpp ik-solvers ct_core ct_rbd ct_optcon)


add_executable(admm-mpc-contact-rcg src/OPT_solvers.cpp src/main_admm_rcg_mpc.cpp)
target_include_directories(admm-mpc-contact-rcg PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(admm-mpc-contact-rcg orocos-kdl admm-solver kuka-models ModernRoboticsCpp ik-solvers)


add_executable(admm-mpc-contact src/OPT_solvers.cpp src/main_admm_mpc.cpp)
target_include_directories(admm-mpc-contact PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(admm-mpc-contact orocos-kdl admm-solver kuka-models ModernRoboticsCpp ik-solvers)

add_executable(ddp-optim src/ddp_main.cpp)
target_include_directories(ddp-optim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ddp-optim orocos-kdl ddp-solver kuka-models cnpy ModernRoboticsCpp ik-solvers)


#add_executable(mpc-optim src/mpc_main.cpp)
#target_include_directories(mpc-optim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#target_link_libraries(mpc-optim orocos-kdl ddp-solver kuka-models cnpy ModernRoboticsCpp ik-solvers)


add_executable(ik-test ik-solvers/ik_test.cpp)
target_include_directories(ik-test PUBLIC ${ModernRoboticsCpp_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ik-test ModernRoboticsCpp ik-solvers)


# test scripts
add_executable(test_models src/main_test_models.cpp)
target_include_directories(test_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_models ModernRoboticsCpp ik-solvers orocos-kdl kuka-models ddp-solver)



###########################################INSTALL AND EXPORT##############################################
include(CMakePackageConfigHelpers)
install (TARGETS kuka-models ik-solvers ddp-solver admm-solver
        EXPORT ADMMTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
# add include directories
# target_include_directories(ddp-admm-solver
#                            PUBLIC
#                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#                            $<INSTALL_INTERFACE:include>
# )

# install header file
install(FILES include/Curvature.hpp include/CostFunctionADMM.hpp include/CostFunctionContact.hpp include/config.h include/RobotDynamics.hpp include/RobotAbstract.h include/SoftContactModel.h include/RobotPlant.hpp include/eigenmvn.hpp include/RobotAbstract.h include/utils.h DESTINATION include)

# # set version
set(version 3.4.1)

# set_property(TARGET ddp-admm-solver kuka-models PROPERTY VERSION ${version})
# set_property(TARGET ddp-admm-solver kuka-models PROPERTY SOVERSION 3)
# set_property(TARGET ddp-admm-solver kuka-models PROPERTY
#   INTERFACE_admm_MAJOR_VERSION 3)
# set_property(TARGET ddp-admm-solver kuka-models APPEND PROPERTY
#   COMPATIBLE_INTERFACE_STRING admm_MAJOR_VERSION
# )

# generate the export targets for the build tree
export(EXPORT ADMMTargets
       FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/ADMMTargets.cmake"
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ADMMConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/ADMMConfig.cmake"
  INSTALL_DESTINATION cmake
  NO_CHECK_REQUIRED_COMPONENTS_MACRO

)

# install config files
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/ADMMConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/ADMMConfigVersion.cmake"
        DESTINATION cmake
)

# generate and install export file
# install(EXPORT ADMMTargets
#         FILE ADMMTargets.cmake
#         DESTINATION cmake
# )
