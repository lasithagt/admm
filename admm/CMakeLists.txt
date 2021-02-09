# create library
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ADMMMultiBlock.cpp ${CMAKE_CURRENT_SOURCE_DIR}/ADMMTrajOptimizer.cpp ${CMAKE_CURRENT_SOURCE_DIR}/ADMMTrajOptimizerMPC.cpp)

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
install(FILES include/ADMMMultiBlock.hpp include/ADMMTrajOptimizer.hpp include/ProjectionOperator.hpp include/admmPublic.hpp include/ADMMTrajOptimizerMPC.hpp include/ModelPredictiveControlADMM.hpp include/IterativeLinearQuadraticRegulatorADMM.hpp include/RobotPublisherMPC.hpp DESTINATION include)

# generate and install export file
install(EXPORT ADMMSolversTargets
        FILE ADMMADMMSolversTargets.cmake
        DESTINATION cmake
)




# # for correct libraries locations across platforms
# include(GNUInstallDirs)

# ## copy the header files
# install(DIRECTORY include/ct/core DESTINATION include/ct)
# install(DIRECTORY examples/include/ct/core DESTINATION include/ct)

# ## copy the cmake files required for find_package()
# install(FILES "cmake/ct_coreConfig.cmake" DESTINATION "share/ct_core/cmake")

# ## install library and targets
# install(
#     TARGETS ct_core ct_plot ${PRESPEC_LIB_NAMES}
#     EXPORT ct_core_export
#     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     )

# ## create the ct_core.cmake file which holds target includes and dependencies
# install (EXPORT ct_core_export DESTINATION share/ct_core/cmake)