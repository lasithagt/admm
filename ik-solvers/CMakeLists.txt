# create library
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ik_solver.cpp)
add_library(ik-solvers STATIC ${SOURCES})


# add_library(MathFunctions::Addition ALIAS Addition)

# add include directories
target_include_directories(ik-solvers
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           $<INSTALL_INTERFACE:include>
)

# install the target and create export-set
install(TARGETS ik-solvers
        EXPORT IKSolversTargets
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/bin
        INCLUDES DESTINATION include
)

# install header file
install(FILES include/ik_solver.hpp include/ik_trajectory.hpp DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/include)

# generate and install export file
install(EXPORT IKSolversTargets
        FILE IKSolversTargets.cmake
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/cmake
)
