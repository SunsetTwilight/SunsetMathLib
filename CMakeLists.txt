cmake_minimum_required(VERSION 3.22)

project(MathLib
    VERSION      1.0.0
    DESCRIPTION  "Header-only C++20 math library for real-time audio (JUCE)"
    LANGUAGES    CXX
)

# ============================================================
#  
# ============================================================
option(MATHLIB_BUILD_TESTS  "Build tests"        OFF)
option(MATHLIB_ENABLE_AVX2  "Enable AVX2 instruction set"   ON)
option(MATHLIB_INSTALL      "Generate install rules"   ON)

# ============================================================
#  Library target(INTERFACE = Header-only)
# ============================================================
add_library(MathLib INTERFACE)
add_library(MathLib::MathLib ALIAS MathLib)

target_include_directories(MathLib
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# C++20 required
target_compile_features(MathLib INTERFACE cxx_std_20)

# AVX2 flags
# MSVC: /O2  /RTC1  INTERFACE flags
# -> AVX2 only set instruction set, optimization level is left to the consumer
if(MATHLIB_ENABLE_AVX2)
    if(MSVC)
        target_compile_options(MathLib INTERFACE /arch:AVX2)
    else()
        target_compile_options(MathLib INTERFACE -mavx2)
    endif()
endif()

# ============================================================
#  Tests
# ============================================================
if(MATHLIB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ============================================================
#  Install
# ============================================================
if(MATHLIB_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    install(
        DIRECTORY   include/MathLib
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    install(
        TARGETS MathLib
        EXPORT  MathLibTargets
    )
    install(
        EXPORT      MathLibTargets
        FILE        MathLibTargets.cmake
        NAMESPACE   MathLib::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MathLib
    )
    configure_package_config_file(
        cmake/MathLibConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/MathLibConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MathLib
    )
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/MathLibConfigVersion.cmake
        VERSION       ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(
        FILES
            ${CMAKE_CURRENT_BINARY_DIR}/MathLibConfig.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/MathLibConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MathLib
    )
endif()
