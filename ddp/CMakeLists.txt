# create library
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ilqrsolver.cpp ../src/kuka_arm.cpp ${CMAKE_CURRENT_SOURCE_DIR}/ddp.cpp)
add_library(ddp-solver STATIC ${SOURCES})


# add_library(MathFunctions::Addition ALIAS Addition)

# add include directories
target_include_directories(ddp-solver
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           $<INSTALL_INTERFACE:include>
)
target_link_libraries(ddp-solver PUBLIC kuka-models cnpy)


# install the target and create export-set
install(TARGETS ddp-solver
        EXPORT DDPSolverTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# install header file
install(FILES include/mpc.hpp include/ddp.h include/cost_function.h include/ilqrsolver.h DESTINATION include)

# generate and install export file
install(EXPORT DDPSolverTargets
        FILE ADMMDDPSolverTargets.cmake
        DESTINATION cmake
)
