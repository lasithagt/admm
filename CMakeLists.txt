cmake_minimum_required(VERSION 3.15)
include(GenerateExportHeader)

project(DDP-ADMM)
SET (CMAKE_CXX_COMPILER "/usr/bin/g++")


if(NOT CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

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


find_package(OROCOS_KDL)
if (OROCOS_KDL)
endif()

# find_package(ZLIB REQUIRED)
find_package(ModernRoboticsCpp)
find_package(CNPY)


# set(OROCOS_KDL_INCLUDE_DIR ${OROCOS_KDL_INCLUDE_DIR})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "-march=native -mtune=native -mavx2 -mfma")
set(CMAKE_CXX_FLAGS "-std=c++11")


include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${OROCOS_KDL_INCLUDE_DIR} ${ModernRoboticsCpp_INCLUDE_DIR} /usr/local/include ${CMAKE_CURRENT_SOURCE_DIR}/ik-solvers/include/)

message(STATUS ${OROCOS_KDL_INCLUDE_DIR})

# find ModernRobotics package
message(STATUS ${ModernRoboticsCpp_INCLUDE_DIR})


add_subdirectory(plant-models)
add_subdirectory(ik-solvers)
add_subdirectory(ddp)


set(SOURCES_DDP_ADMM ${CMAKE_CURRENT_SOURCE_DIR}/src/ilqrsolver_admm.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/kuka_arm.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/admm.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/admm_main.cpp)
add_library(ddp-admm-solver ${SOURCES_DDP_ADMM})
target_include_directories(ddp-admm-solver PUBLIC include ${ModernRoboticsCpp_INCLUDE_DIR} ${CNPY_INCLUDE_DIR})
target_link_libraries(ddp-admm-solver PUBLIC kuka-models cnpy)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# EXECUTABLES
#######################################################################################

add_executable(admm-contact src/OPT_solvers.cpp)
target_include_directories(admm-contact PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(admm-contact orocos-kdl ddp-admm-solver kuka-models ModernRoboticsCpp ik-solvers)


add_executable(ddp-optim src/ddp_main.cpp)
target_include_directories(ddp-optim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ddp-optim orocos-kdl ddp-solver kuka-models cnpy ModernRoboticsCpp ik-solvers)


add_executable(mpc-optim src/mpc_main.cpp)
target_include_directories(mpc-optim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mpc-optim orocos-kdl ddp-solver kuka-models cnpy ModernRoboticsCpp ik-solvers)


add_executable(admm-mpc src/admm_mpc_main.cpp)
target_include_directories(admm-mpc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(admm-mpc orocos-kdl ddp-admm-solver kuka-models ModernRoboticsCpp ik-solvers cnpy)


add_executable(ik-test ik-solvers/ik_test.cpp)
target_include_directories(ik-test PUBLIC ${ModernRoboticsCpp_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ik-test ModernRoboticsCpp ik-solvers)


# test scripts
add_executable(test_cases src/test.cpp)
target_include_directories(test_cases PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_cases ModernRoboticsCpp ik-solvers orocos-kdl kuka-models ddp-solver)


add_executable(test_hdf5 src/testhdf5.cpp)
target_include_directories(test_hdf5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cnpy)
target_link_libraries(test_hdf5 cnpy)



############################################INSTALL AND EXPORT##############################################
include(CMakePackageConfigHelpers)
install (TARGETS ddp-admm-solver kuka-models
        EXPORT ADMMTargets
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib/static
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/bin
        INCLUDES DESTINATION include)

# # set version
set(version 3.4.1)

set_property(TARGET ddp-admm-solver kuka-models PROPERTY VERSION ${version})
set_property(TARGET ddp-admm-solver kuka-models PROPERTY SOVERSION 3)
set_property(TARGET ddp-admm-solver kuka-models PROPERTY
  INTERFACE_admm_MAJOR_VERSION 3)
set_property(TARGET ddp-admm-solver kuka-models APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING admm_MAJOR_VERSION
)

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
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/ADMMConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/ADMMConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/cmake
)

# install config files
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/ADMMConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/ADMMConfigVersion.cmake"
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/cmake
)

