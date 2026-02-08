cmake_minimum_required(VERSION 3.16)
project(RobotMotionCore VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOT_MOTION_CORE_BUILD_EXAMPLES "Build examples" ON)
option(ROBOT_MOTION_CORE_BUILD_TESTS "Build tests" ON)
option(ROBOT_MOTION_CORE_BUILD_PYTHON "Build Python bindings (pybind11)" ON)
option(ROBOT_MOTION_CORE_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(ROBOT_MOTION_CORE_ENABLE_SANITIZERS "Enable sanitizers (Debug only)" OFF)

find_package(Eigen3 REQUIRED)

add_library(robot_motion_core
    src/robot_arm.cpp
    src/ik_solver_dls.cpp
)
add_library(robot_motion_core::robot_motion_core ALIAS robot_motion_core)

set_target_properties(robot_motion_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_features(robot_motion_core PUBLIC cxx_std_17)

target_include_directories(robot_motion_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(robot_motion_core PUBLIC Eigen3::Eigen)

if(ROBOT_MOTION_CORE_BUILD_PYTHON)
    # Use a stable tagged release.
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.13.6
    )
    FetchContent_MakeAvailable(pybind11)

    pybind11_add_module(industrial_robot_core src/python_bindings.cpp)
    target_link_libraries(industrial_robot_core PRIVATE robot_motion_core::robot_motion_core)
    target_compile_features(industrial_robot_core PRIVATE cxx_std_17)
endif()

if(ROBOT_MOTION_CORE_ENABLE_WARNINGS)
    if(MSVC)
        target_compile_options(robot_motion_core PRIVATE /W4)
    else()
        target_compile_options(robot_motion_core PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

if(ROBOT_MOTION_CORE_ENABLE_SANITIZERS)
    if(NOT MSVC)
        target_compile_options(robot_motion_core PRIVATE -fsanitize=address,undefined)
        target_link_options(robot_motion_core PRIVATE -fsanitize=address,undefined)
    endif()
endif()

if(ROBOT_MOTION_CORE_BUILD_EXAMPLES)
    add_executable(robot_demo examples/main.cpp)
    target_link_libraries(robot_demo PRIVATE robot_motion_core::robot_motion_core)
endif()

if(ROBOT_MOTION_CORE_BUILD_TESTS)
    enable_testing()

    add_executable(test_fk tests/test_fk.cpp)
    target_link_libraries(test_fk PRIVATE robot_motion_core::robot_motion_core)
    add_test(NAME robot_motion_core.test_fk COMMAND test_fk)

    add_executable(test_jacobian_fd tests/test_jacobian_fd.cpp)
    target_link_libraries(test_jacobian_fd PRIVATE robot_motion_core::robot_motion_core)
    add_test(NAME robot_motion_core.test_jacobian_fd COMMAND test_jacobian_fd)

    add_executable(test_ik_irb120 tests/test_ik_irb120.cpp)
    target_link_libraries(test_ik_irb120 PRIVATE robot_motion_core::robot_motion_core)
    add_test(NAME robot_motion_core.test_ik_irb120 COMMAND test_ik_irb120)

    add_executable(test_ik_joint_limits tests/test_ik_joint_limits.cpp)
    target_link_libraries(test_ik_joint_limits PRIVATE robot_motion_core::robot_motion_core)
    add_test(NAME robot_motion_core.test_ik_joint_limits COMMAND test_ik_joint_limits)

    add_executable(test_ik_near_singular tests/test_ik_near_singular.cpp)
    target_link_libraries(test_ik_near_singular PRIVATE robot_motion_core::robot_motion_core)
    add_test(NAME robot_motion_core.test_ik_near_singular COMMAND test_ik_near_singular)

    add_executable(benchmark_ik tests/benchmark_ik.cpp)
    target_link_libraries(benchmark_ik PRIVATE robot_motion_core::robot_motion_core)
endif()

# -----------------
# Install + export
# -----------------
install(TARGETS robot_motion_core
    EXPORT robot_motion_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT robot_motion_coreTargets
    FILE robot_motion_coreTargets.cmake
    NAMESPACE robot_motion_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_motion_core
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/robot_motion_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/robot_motion_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/robot_motion_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_motion_core
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/robot_motion_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/robot_motion_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_motion_core
)
