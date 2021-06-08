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

set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "" FORCE)

option(BUILD_TESTS "Compile tests" ON)
option(BUILD_TESTS_DEVEL "Compile tests devel branch" ON)

add_library(sisl_toolbox SHARED
    src/curve.cpp
    src/generic_curve.cpp
    src/straight_line.cpp
    src/circle.cpp
    src/path.cpp
    src/persistence_manager.cpp
    src/path_factory.cpp
)


target_link_libraries(sisl_toolbox sisl jsoncpp Geographic)

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

    add_executable(test_serpentine_movePoint test/test_serpentine_movePoint.cpp)
    target_link_libraries(test_serpentine_movePoint sisl_toolbox)

    #add_executable(test_serpentine_extractSection test/test_serpentine_extractSection.cpp)
    #target_link_libraries(test_serpentine_extractSection sisl_toolbox)

    add_executable(test_serpentine_generation test/test_serpentine_generation.cpp)
    target_link_libraries(test_serpentine_generation sisl_toolbox)


    add_test(test_hippodrome test_serpentine test_serpentine_movePoint test_serpentine_extractSection test_serpentine_generation)
endif(BUILD_TESTS)
