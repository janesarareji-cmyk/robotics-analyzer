include(CTest)

# test_placeholder — R0.2
add_executable(test_placeholder test_placeholder.cpp)
target_link_libraries(test_placeholder PRIVATE core_lib)
target_compile_features(test_placeholder PUBLIC cxx_std_20)
add_test(NAME test_placeholder COMMAND test_placeholder)

# test_eigen — R0.5
add_executable(test_eigen test_eigen.cpp)
target_link_libraries(test_eigen PRIVATE core_lib Eigen3::Eigen)
target_compile_features(test_eigen PUBLIC cxx_std_20)
add_test(NAME test_eigen COMMAND test_eigen)

# test_cuda — R0.6
add_executable(test_cuda test_cuda.cpp)
target_link_libraries(test_cuda PRIVATE cuda_lib)
target_include_directories(test_cuda PRIVATE ${CMAKE_SOURCE_DIR}/cuda)
add_test(NAME test_cuda COMMAND test_cuda)

# test_urm — R2.1
add_executable(test_urm test_urm.cpp)
target_link_libraries(test_urm PRIVATE core_lib)
target_compile_features(test_urm PUBLIC cxx_std_20)
add_test(NAME test_urm COMMAND test_urm)

# test_fk — R2.5
add_executable(test_fk test_fk.cpp)
target_link_libraries(test_fk PRIVATE core_lib Eigen3::Eigen)
target_compile_features(test_fk PUBLIC cxx_std_20)
add_test(NAME test_fk COMMAND test_fk)

# test_compiled - R3.2
add_executable(test_compiled test_compiled.cpp)
target_link_libraries(test_compiled PRIVATE core_lib Eigen3::Eigen)
target_compile_features(test_compiled PUBLIC cxx_std_20)
add_test(NAME test_compiled COMMAND test_compiled)

# test_compiled_arrays - R4.1
add_executable(test_compiled_arrays test_compiled_arrays.cpp)
target_link_libraries(test_compiled_arrays PRIVATE core_lib Eigen3::Eigen)
target_compile_features(test_compiled_arrays PUBLIC cxx_std_20)
add_test(NAME test_compiled_arrays COMMAND test_compiled_arrays)

# test_dynamics - R4.6
add_executable(test_dynamics test_dynamics.cpp)
target_link_libraries(test_dynamics PRIVATE core_lib Eigen3::Eigen)
target_compile_features(test_dynamics PUBLIC cxx_std_20)
add_test(NAME test_dynamics COMMAND test_dynamics)
