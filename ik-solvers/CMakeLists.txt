# create library
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ik_solver.cpp)
add_library(ik-solvers STATIC ${SOURCES})
target_link_libraries(ik-solvers PUBLIC ModernRoboticsCpp)


# add include directories
target_include_directories(ik-solvers
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           $<INSTALL_INTERFACE:include>
)

# install the target and create export-set
install(TARGETS ik-solvers
        EXPORT IKSolversTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# install header file
install(FILES include/ik_solver.hpp include/ik_trajectory.hpp DESTINATION include)

# generate and install export file
install(EXPORT IKSolversTargets
        FILE ADMMIKSolversTargets.cmake
        DESTINATION cmake
)
