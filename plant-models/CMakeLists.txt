# create library
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/KukaModel.cpp ${CMAKE_CURRENT_SOURCE_DIR}/models.cpp)
add_library(kuka-models STATIC ${SOURCES})

# add_library(MathFunctions::Addition ALIAS Addition)

# add include directories
target_include_directories(kuka-models
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/KDL>"
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Screws>"
                           $<INSTALL_INTERFACE:include>
)

find_package(ct_core)
find_package(ct_rbd)

# test scripts
add_executable(test_rbd_models main_test_models.cpp models.cpp KukaModel.cpp)
target_include_directories(test_rbd_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/KDL ${CMAKE_CURRENT_SOURCE_DIR}/RobCodGen)
target_link_libraries(test_rbd_models ModernRoboticsCpp orocos-kdl ct_core ct_rbd)


# install the target and create export-set
install(TARGETS kuka-models
        EXPORT PlantModelTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include KDL Screws
)

# install header file
install(FILES KDL/KukaModel.h KDL/models.h Screws/KukaKinematicsScrews.hpp DESTINATION include)

# generate and install export file
install(EXPORT PlantModelTargets
        FILE ADMMPlantModelTargets.cmake
        DESTINATION cmake
)
