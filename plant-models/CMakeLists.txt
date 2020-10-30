# create library
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/KukaModel.cpp ${CMAKE_CURRENT_SOURCE_DIR}/models.cpp)
add_library(kuka-models STATIC ${SOURCES})

# add_library(MathFunctions::Addition ALIAS Addition)

# add include directories
target_include_directories(kuka-models
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           $<INSTALL_INTERFACE:include>
)

# install the target and create export-set
install(TARGETS kuka-models
        EXPORT PlantModelTargets
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/bin
        INCLUDES DESTINATION include
)

# install header file
# install(FILES Addition.h DESTINATION include)

# generate and install export file
install(EXPORT PlantModelTargets
        FILE PlantModelTargets.cmake
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/cmake
)
