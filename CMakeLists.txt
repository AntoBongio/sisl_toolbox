# SISL Toolbox: Augmenting the SISL library

# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)


# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Define library. Only source files here!
project(sisl_toolbox VERSION 0.1 LANGUAGES CXX)

#set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

option(BUILD_TESTS "Compile tests" ON)
option(BUILD_TESTS_DEVEL "Compile tests devel branch" ON)

add_library(sisl_toolbox SHARED
    src/curve.cpp
    src/generic_curve.cpp
    src/straight_line.cpp
    src/circular_arc.cpp
    src/path.cpp
    src/persistence_manager.cpp
    src/path_factory.cpp
)


target_link_libraries(sisl_toolbox sisl)

set_target_properties(sisl_toolbox PROPERTIES LINKER_LANGUAGE CXX)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(sisl_toolbox PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/sisl_toolbox>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(sisl_toolbox
    PUBLIC
        cxx_auto_type
        cxx_range_for
        cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
#target_link_libraries(lib
#    boost
#    MyOtherLibrary)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS sisl_toolbox EXPORT sisl_toolbox_config
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/sisl_toolbox DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT sisl_toolbox_config DESTINATION share/sisl_toolbox/cmake)

# This makes the project importable from the build directory
export(TARGETS sisl_toolbox FILE sisl_toolbox_config.cmake)


if(BUILD_TESTS)
    
    add_executable(test_hippodrome test/test_hippodrome.cpp)
    target_link_libraries(test_hippodrome sisl_toolbox)

    add_executable(test_polygon test/test_polygon.cpp)
    target_link_libraries(test_polygon sisl_toolbox)

    add_executable(test_spiral test/test_spiral.cpp)
    target_link_libraries(test_spiral sisl_toolbox)

    add_executable(test_serpentine test/test_serpentine.cpp)
    target_link_libraries(test_serpentine sisl_toolbox)

    add_executable(test_race_track test/test_race_track.cpp)
    target_link_libraries(test_race_track sisl_toolbox)


    add_test(test_hippodrome test_polygon test_spiral test_serpentine test_race_track)
endif(BUILD_TESTS)
