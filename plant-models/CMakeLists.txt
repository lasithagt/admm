# create library
find_package(ct_core)
find_package(ct_rbd)

set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/KDL/KukaModel.cpp ${CMAKE_CURRENT_SOURCE_DIR}/RobCodGen/RobCodGenModel.cpp)
add_library(kuka-models STATIC ${SOURCES})
target_link_libraries(kuka-models ct_core ct_rbd)

# add include directories
target_include_directories(kuka-models
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/KDL>"
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Screws>"
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/RobCodGen>"
                           $<INSTALL_INTERFACE:include>
)



# # test scripts
add_executable(test_rbd_models main_test_models.cpp)
target_include_directories(test_rbd_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/KDL ${CMAKE_CURRENT_SOURCE_DIR}/RobCodGen)
target_link_libraries(test_rbd_models kuka-models ModernRoboticsCpp orocos-kdl)


# install the target and create export-set
install(TARGETS kuka-models
        EXPORT PlantModelTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include KDL Screws
)

# install header file
install(FILES KDL/KukaModel.h KDL/models.h RobCodGen/RobCodGenModel.h Screws/KukaKinematicsScrews.hpp DESTINATION include)

# generate and install export file
install(EXPORT PlantModelTargets
        FILE ADMMPlantModelTargets.cmake
        DESTINATION cmake
)
