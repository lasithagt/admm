CMAKE_MINIMUM_REQUIRED(VERSION 3.0 FATAL_ERROR)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(CNPY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(ENABLE_STATIC "Build static (.a) library" ON)

find_package(ZLIB REQUIRED)

include_directories(${ZLIB_INCLUDE_DIRS})

add_library(cnpy SHARED "cnpy.cpp")
target_link_libraries(cnpy ${ZLIB_LIBRARIES})
# install(TARGETS "cnpy" LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

if(ENABLE_STATIC)
    add_library(cnpy-static STATIC "cnpy.cpp")
    set_target_properties(cnpy-static PROPERTIES OUTPUT_NAME "cnpy")
    install(TARGETS "cnpy-static" ARCHIVE DESTINATION lib)
endif(ENABLE_STATIC)

# install(FILES "cnpy.h" DESTINATION include)
# install(FILES "mat2npz" "npy2mat" "npz2mat" DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

add_executable(example1 example1.cpp)
target_link_libraries(example1 cnpy)




install (TARGETS cnpy
        EXPORT CNPYTargets
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/lib
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/bin
        INCLUDES DESTINATION include)

# include CMakePackageConfigHelpers macro
include(CMakePackageConfigHelpers)

# set version
set(version 3.4.1)

set_property(TARGET cnpy PROPERTY VERSION ${version})
set_property(TARGET cnpy PROPERTY SOVERSION 3)
set_property(TARGET cnpy PROPERTY
  INTERFACE_cnpy_MAJOR_VERSION 3)
set_property(TARGET cnpy APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING cnpy_MAJOR_VERSION
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/CNPYConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/CNPYConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/CNPYConfig.cmake"
  INSTALL_DESTINATION lib/cmake
)

install(FILES cnpy.h DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/include)


# install config files
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/CNPYConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/CNPYConfigVersion.cmake"
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/cmake
)

# generate the export targets for the build tree
export(EXPORT CNPYTargets
       FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/CNPYTargets.cmake"
       

)


# generate and install export file
install(EXPORT CNPYTargets
        FILE CNPYTargets.cmake
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install/cmake
)