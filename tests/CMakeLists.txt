cmake_minimum_required(VERSION 3.22)

# ============================================================
#  Tests
# ============================================================
if(MSVC)
    # /O2  Debug  /RTC1 
    # flags
    set(MATHLIB_TEST_COMPILE_OPTIONS
        /W4
        /wd4100   # Suppress unused parameter warnings
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:RelWithDebInfo>:/O2>
        $<$<CONFIG:Debug>:/Od>
    )
else()
    set(MATHLIB_TEST_COMPILE_OPTIONS
        -O2
        -fno-stack-protector
        -Wall
        -Wextra
        -Wno-unused-parameter
    )
endif()

# Tests
function(mathlib_add_test TARGET_NAME SOURCE_FILE)
    add_executable(${TARGET_NAME} ${SOURCE_FILE})
    target_link_libraries(${TARGET_NAME} PRIVATE MathLib::MathLib)
    target_compile_options(${TARGET_NAME} PRIVATE ${MATHLIB_TEST_COMPILE_OPTIONS})
    add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
endfunction()

# ============================================================
#  Tests
# ============================================================
mathlib_add_test(test_all          test_all.cpp)
mathlib_add_test(test_bitop        test_bitop.cpp)
mathlib_add_test(test_vecmath      test_vecmath.cpp)
mathlib_add_test(test_vector2      test_vector2.cpp)
mathlib_add_test(test_vector34     test_vector34.cpp)
mathlib_add_test(test_matn         test_matn.cpp)
mathlib_add_test(test_quaternion   test_quaternion.cpp)
mathlib_add_test(test_math_h       test_math_h.cpp)
mathlib_add_test(test_new_features  test_new_features.cpp)
mathlib_add_test(test_transform  test_transform.cpp)
mathlib_add_test(test_mathutils  test_mathutils.cpp)
mathlib_add_test(test_geometry  test_geometry.cpp)
mathlib_add_test(test_frustum  test_frustum.cpp)
mathlib_add_test(test_triangle  test_triangle.cpp)
mathlib_add_test(test_obb  test_obb.cpp)
