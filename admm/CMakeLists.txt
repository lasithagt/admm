# create library
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ilqrsolver_admm.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/kuka_arm.cpp ${CMAKE_CURRENT_SOURCE_DIR}/admm.cpp ${CMAKE_CURRENT_SOURCE_DIR}/ADMMTrajOptimizer.cpp ${CMAKE_CURRENT_SOURCE_DIR}/ADMMTrajOptimizerMPC.cpp)
add_library(admm-solver STATIC ${SOURCES})
target_link_libraries(admm-solver PUBLIC kuka-models ik-solvers cnpy ModernRoboticsCpp)


# add include directories
target_include_directories(admm-solver
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           $<INSTALL_INTERFACE:include>
)

# install the target and create export-set
install(TARGETS admm-solver
        EXPORT ADMMSolversTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# install header file
install(FILES include/admm.hpp include/ADMMTrajOptimizer.hpp include/ADMMTrajOptimizerMPC.hpp include/mpc.hpp include/mpc_admm.hpp include/ilqrsolver_admm.hpp DESTINATION include)

# generate and install export file
install(EXPORT ADMMSolversTargets
        FILE ADMMADMMSolversTargets.cmake
        DESTINATION cmake
)
